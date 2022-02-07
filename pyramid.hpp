#ifndef PRIMITIVES_PYRAMID_HPP
#define PRIMITIVES_PYRAMID_HPP

#include "Primitives/Headers/ImageView.hpp"
#include <vector>
#include <functional>

class ImageView;
class Pyramid;
struct LevelKeeper;

using BuildFunction = std::function<void(Pyramid&, ImageView&&, size_t)>;

using Preprocess = std::function<void(LevelKeeper& )>;

using ItPyramid = std::vector<LevelKeeper>::iterator;
using constItPyramid = std::vector<LevelKeeper>::const_iterator;

class Pyramid{
public:
    Pyramid() = delete;
    Pyramid(const Pyramid&) = default;
    Pyramid(Pyramid&&) = default;
    Pyramid& operator=(const Pyramid&) = default;


    /**
    * @brief
    * Initialization pyramid
    *
    * @param src_image first level pyramid
    * @param levels set pyramid level
    * @param scale set pyramid scale step
    * @param II initialization of intagral image
    * @param sqr_II initialization of squared integral image
    */
    Pyramid(const ImageView& null_level, size_t levels, float scale,
            bool II = false, bool sqr_II = false);

    /**
    * @brief
    * Initialization pyramid
    *
    * @param src_image first level pyramid
    * @param levels set pyramid level
    * @param scale set pyramid scale step
    * @param II initialization of intagral image
    * @param sqr_II initialization of squared integral image
    */
    explicit
    Pyramid(const ImageView& null_level, float scale = 2,
            bool II = false, bool sqr_II = false);

    /**
    * @brief
    * Initialization pyramid
    *
    * @param null_level first level pyramid
    * @param levels set pyramid level
    * @param scale set pyramid scale step
    * @param Preprocess for preprocessing images
    * @param II initialization of intagral image
    * @param sqr_II initialization of squared integral image
    */
    Pyramid(const ImageView& null_level, size_t levels, float scale,
            Preprocess preprocess, bool II = false, bool sqr_II = false);

    /**
    * @brief Getting an image from a pyramid
    *
    * @param level required pyramid level
    * */
    [[nodiscard]] const ImageView& getImage(size_t level) const;

    /**
    * @brief Getting an intagral image from a pyramid
    *
    * @param level required pyramid level
    * */
    [[nodiscard]] const std::optional<ImageView>& getII(size_t level) const;

    /**
    * @brief Getting an squared integral image from a pyramid
    *
    * @param level required pyramid level
    * */
    [[nodiscard]] const std::optional<ImageView>& getSqrII(size_t level) const;

    [[nodiscard]] constItPyramid cbegin() const;
    [[nodiscard]] constItPyramid cend() const;

    /**
    * @brief Refresh all pyramid levels
    *
    * @param null_level first level pyramid
    * */
    void update(const ImageView& null_level);

    /**
    * @brief Refresh all pyramid levels
    *
    * @param null_level first level pyramid
    * @param ROI where will the pyramid update take place
    * */
    void update(const ImageView& null_level, strobeRectangle<int32_t> ROI);

    /**
    * @brief Refresh all pyramid levels
    *
    * @param preprocess for preprocessing images
    * */
    void update(Preprocess preprocess);

    /**
    * @return the scale with which the pyramid works for each level
    * */
    [[nodiscard]] float scale() const;

    /**
    * @return number of pyramid levels
    * */
    [[nodiscard]] size_t levels() const;

    /**
    * @brief sets a new scale and rebuilds the pyramid
    * @param scale the scale with which the pyramid works for each level
    * */
    void setScale(float scale);

    /**
    * @brief sets a new levels and rebuilds the pyramid
    * @param levels_num the levels with which the pyramid works for each level
    * */
    void setLevels(size_t levels_num);

    /**
    * @brief includes the construction of an integral image
    * */
    void setII();

    /**
    * @brief includes the construction of an squared integral image
    * */
    void setSqrII();

    /**
    * @brief turns off integral imaging
    * */
    void disableII();

    /**
    * @brief turns off squared integral image
    * */
    void disableSqrII();

    void copy(Pyramid &other);

private:
    void build(const ImageView& null_level, const BuildFunction& callback);
    void buildLUT();

    void initLevel(ImageView&& new_level, size_t level);
    void reInitLevel(ImageView&& new_level, size_t level);

    bool isParamsAcceptable(const ImageView& null_level) const;
    size_t maxAcceptableLevel(const ImageView& null_level) const;
    size_t resultReverseScale(const ImageView& null_level) const;

    ItPyramid begin();
    ItPyramid end();

    std::vector<LevelKeeper> pyramid_;
    size_t levels_ = 4;
    float scale_ = 2.0f;

    bool enableII_ = false;
    bool enableSqrII_ = false;

    Preprocess preprocess_;

    bool enableLUT_ = true;
};

class LevelKeeper
{
public:
    explicit LevelKeeper(ImageView&& image) :
        image_(image) {}

    LevelKeeper(ImageView image) :
        image_(image) {}

    void buildLUT(float scale);
    void buildII(const ImageView& image);
    void buildSqrII(const ImageView& image);

    LevelKeeper copy() const;

    ImageView image_;
    std::optional<ImageView> LUT_;
    std::optional<ImageView> II_;
    std::optional<ImageView> sqr_II_;
};


#endif //PRIMITIVES_PYRAMID_HPP
