#include <containers/SemanticColor.h>

using namespace uima;
using namespace rs_log_learn;

SemanticColor::SemanticColor(iai_rs::SemanticColor sColor)
{
    std::vector<std::string> colors = sColor.color.get();
    std::vector<float> ratios = sColor.ratio.get();

    for(int i = 0; i < colors.size(); ++i)
    {
        sColorMapping[colors[i]] = ratios[i];
    }
}

SemanticColor::~SemanticColor()
{
}

std::map<std::string, float> SemanticColor::getColorMapping()
{
    return sColorMapping;
}

