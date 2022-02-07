#include "ImageProcessing/Headers/image_processing.h"
#include "ImageProcessing/Headers/MathProcessing.h"

#include "Primitives/Headers/pyramid.hpp"

#include "ImageProcessing/Headers/imageProcessingToolkit_CPU.h"

#include <utility>

Pyramid::Pyramid(const ImageView& null_level, size_t levels, float scale, bool II, bool sqr_II) :
    levels_(levels), scale_(scale), enableII_(II), enableSqrII_(sqr_II)
{
    build(null_level, &Pyramid::initLevel);
}

Pyramid::Pyramid(const ImageView& null_level, float scale, bool II, bool sqr_II) :
    scale_(scale), enableII_(II), enableSqrII_(sqr_II)
{
    levels_ = maxAcceptableLevel(null_level);
    build(null_level, &Pyramid::initLevel);
}

Pyramid::Pyramid(const ImageView& null_level, size_t levels, float scale, Preprocess preprocess, bool II, bool sqr_II) :
    levels_(levels), scale_(scale), preprocess_(std::move(preprocess)), enableII_(II), enableSqrII_(sqr_II)
{
    build(null_level, &Pyramid::initLevel);
}

bool Pyramid::isParamsAcceptable(const ImageView& null_level) const
{
    return levels_ <= maxAcceptableLevel(null_level);
}

size_t Pyramid::maxAcceptableLevel(const ImageView& null_level) const
{
    size_t N = std::min(null_level.width(), null_level.height());
    return std::log(static_cast<float>(N)) / std::log(scale_)/* + 1*/; // не учитываем уровень 1x1
}

size_t Pyramid::resultReverseScale(const ImageView& null_level) const
{
    return null_level.width() * std::pow(scale_, levels_ - 1);
}

void Pyramid::build(const ImageView& null_level, const BuildFunction& build_functor)
{
    try{
        if(scale_ >= 1 && !isParamsAcceptable(null_level))
            throw std::invalid_argument("\nInvalid scale/size ratio.\n");
        if(scale_ < 1 && resultReverseScale(null_level) > 10000)
            throw std::invalid_argument("\nInvalid scale/size ratio for reverse pyramid.\n");
        if(!build_functor)
            throw std::domain_error("\nBuilding pointer missing.\n");
        if(levels_ == 0){
            throw std::invalid_argument("\nInvalid num of levels\n");
        }
    }
    catch(std::invalid_argument& ex){
        std::cerr << ex.what();
    }
    catch(std::domain_error& ex){
        std::cerr << ex.what();
    }

    build_functor(*this, extractGrayscale<uint8_t>(null_level), 0);
    for(size_t current_level = 0; current_level < levels_ - 1; ++current_level)
    {
        size_t scaled_w = getImage(current_level).width() / scale_;
        size_t scaled_h = getImage(current_level).height() / scale_;

        ImageView resized_image;

        if(scale_ == ceilf(scale_))
            resized_image = resizeBilinearForIntScales<uint8_t>(getImage(current_level), scaled_w, scaled_h);
        else
        {
            if(enableLUT_)
                resized_image = resizeBilinearLUT<uint8_t>(getImage(current_level), *pyramid_.at(current_level).LUT_);
            else
            {
                #ifdef USE_OPENMP
                    resized_image = resizeBilinearParallel<uint8_t>(getImage(current_level), scaled_w, scaled_h);
                #else
                    resized_image = resizeBilinearSerial<uint8_t>(getImage(current_level), scaled_w, scaled_h);
                #endif
            }
        }

        build_functor(*this, std::move(resized_image), current_level + 1);
    }
    update(preprocess_);
}

void Pyramid::update(const ImageView& null_level)
{
    if((null_level.width() != getImage(0).width()) && (null_level.height() != getImage(0).height()))
        buildLUT(); /// re-build if dimension changed

    build(null_level, &Pyramid::reInitLevel);
}

void Pyramid::update(Preprocess preprocess)
{
    preprocess_ = std::move(preprocess);

    std::for_each(begin(), end(),
    [this](LevelKeeper& level)
    {
        if(preprocess_)
            preprocess_(level);
        if(enableII_)
            level.buildII(level.image_);
        if(enableSqrII_)
            level.buildSqrII(level.image_);
    });
}


void Pyramid::update(const ImageView& null_level, strobeRectangle<int32_t> ROI){
    begin()->image_ = std::forward<const ImageView>(null_level);
    auto prev_level = begin();
    std::for_each(begin() + 1, end(),
    [&](LevelKeeper& level) -> void
    {
        ROI.scale(scale());
        ROI.cropByFrame(level.image_.width(), level.image_.height());
        resize_bilinear_with_LUT_in_roi<uint8_t, uint8_t>(
            prev_level->image_.ptr<uint8_t>(), prev_level->image_.width(), prev_level->image_.height(),
            ROI.bottomLeft().x(), ROI.bottomLeft().y(), ROI.width(), ROI.height(),
            level.image_.ptr<uint8_t>(), level.image_.width(), level.image_.height(), prev_level->LUT_->ptr<int32_t>());
        ++prev_level;
    });
}


void Pyramid::buildLUT()
{
    std::for_each(begin(), end(),
        [this](LevelKeeper& level)
            {level.buildLUT(scale_);});
}

void Pyramid::initLevel(ImageView&& new_level, size_t level)
{
    pyramid_.push_back(std::move(new_level));
    pyramid_.at(level).buildLUT(scale_);
}

void Pyramid::reInitLevel(ImageView&& new_level, size_t level)
{
    pyramid_.at(level).image_ = std::move(new_level);
    pyramid_.at(level).buildLUT(scale_);
}

size_t Pyramid::levels() const
{
    return levels_;
}

float Pyramid::scale() const
{
    return scale_;
}

void Pyramid::setScale(float scale)
{
    if(scale_ != scale)
    {
        scale_ = scale;
        build(getImage(0), &Pyramid::reInitLevel);
    }
}

void Pyramid::setLevels(size_t levels_num)
{
    if(levels_ != levels_num)
    {
        levels_ = levels_num;
        build(getImage(0), &Pyramid::reInitLevel);
    }
}

void Pyramid::setII()
{
    if(!enableII_)
    {
        enableII_ = true;
        build(getImage(0), &Pyramid::reInitLevel);
    }
}

void Pyramid::setSqrII()
{
    if(!enableSqrII_)
    {
        enableSqrII_ = true;
        build(getImage(0), &Pyramid::reInitLevel);
    }
}

void Pyramid::disableII()
{
    if(enableII_)
        enableII_ = false;
}

void Pyramid::disableSqrII()
{
    if(enableSqrII_)
        enableSqrII_ = false;
}

void Pyramid::copy(Pyramid &other)
{
    levels_ = other.levels_;
    scale_ = other.scale_;

    for (int32_t i = levels_ - 1; i >= 0; --i)
    {
        pyramid_[i] = other.pyramid_[i].copy();
    }
}

const ImageView& Pyramid::getImage(size_t level) const
{
    return pyramid_.at(level).image_;
}

const std::optional<ImageView>& Pyramid::getII(size_t level) const
{
    return pyramid_.at(level).II_;
}

const std::optional<ImageView>& Pyramid::getSqrII(size_t level) const
{
    return pyramid_.at(level).sqr_II_;
}

constItPyramid Pyramid::cbegin() const
{
    return pyramid_.cbegin();
}

constItPyramid Pyramid::cend() const
{
    return pyramid_.cend();
}

ItPyramid Pyramid::begin()
{
    return pyramid_.begin();
}

ItPyramid Pyramid::end()
{
    return pyramid_.end();
}

void LevelKeeper::buildLUT(float scale)
{
    LUT_.emplace(makeBilinearWeightsLUT<int32_t>(image_.width(), image_.height(), scale, scale));
}

void LevelKeeper::buildII(const ImageView& image)
{
    II_.emplace(computeII<uint8_t>(image));
}

void LevelKeeper::buildSqrII(const ImageView& image)
{
    sqr_II_.emplace(computeSqrII<uint8_t>(image));
}

LevelKeeper LevelKeeper::copy() const
{
    LevelKeeper result(image_);

    result.image_ = image_.copy();

    if(LUT_.has_value())
        result.LUT_ = LUT_->copy();

    if(II_.has_value())
        result.II_ = II_->copy();

    if(sqr_II_.has_value())
        result.sqr_II_ = sqr_II_->copy();

    return result;
}
