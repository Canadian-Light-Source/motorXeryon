#ifndef XERYON_STAGES_H
#define XERYON_STAGES_H

#include <cmath>
#include <memory>
#include <unordered_map>

struct XeryonStage
{
    bool isLinear;
    std::string encoderResCmd;
    double encoderRes; // nm/step or deg/step
    uint velocityFactor; // velocity is set in 1 um/s for linear actuators and 0.01 deg/s for angular
    XeryonStage(bool isLinear_, std::string encoderResCmd_, double encoderRes_, uint velocityFactor_)
        : isLinear{isLinear_}, encoderResCmd{encoderResCmd_}, encoderRes{encoderRes_}, velocityFactor{velocityFactor_} {}
};

class XeryonStages
{
public:
    XeryonStages(){};

    /**
     * @brief Returns the stage shared_ptr under the provided name, else the default stage is returned.
     *
     * @param[in] type Stage type.
     */
    std::shared_ptr<XeryonStage> getStage(const std::string &type)
    {
        try
        {
            return stages.at(type);
        }
        catch (const std::exception &e)
        {
            std::cerr << "undefined stage type >> " << type << " <<\n"
                      << "defaulting to linear stage w/o scaling\n";
            return std::make_shared<XeryonStage>(true, "", 1, 1);
        }
    }

private:
    const double pi = acos(-1);
    /**
     * @brief List of available stages.
     * @todo add all stages from the vendor code
     */
    std::unordered_map<std::string, std::shared_ptr<XeryonStage>> stages = {
        {"XLS_312_3N", std::make_shared<XeryonStage>(true, "XLS3=312", 312.5, 1000)},
        {"XRTU_30_3", std::make_shared<XeryonStage>(false, "XRT1=3", (360. / 1843200.), 100)}};
};

#endif // XERYON_STAGES_H
