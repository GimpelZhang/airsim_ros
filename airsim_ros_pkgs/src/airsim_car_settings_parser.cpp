#include "airsim_car_settings_parser.h"

AirSimCarSettingsParser::AirSimCarSettingsParser()
{
    success_ = initializeSettings();
}

bool AirSimCarSettingsParser::success()
{
    return success_;
}

bool AirSimCarSettingsParser::readSettingsTextFromFile(std::string settingsFilepath, std::string& settingsText) 
{
    // check if path exists
    bool found = std::ifstream(settingsFilepath.c_str()).good(); 
    if (found)
    {
        std::ifstream ifs(settingsFilepath);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        // todo airsim's simhud.cpp does error checking here
        settingsText = buffer.str(); // todo convert to utf8 as done in simhud.cpp?
    }

    return found;
}

bool AirSimCarSettingsParser::getSettingsText(std::string& settingsText) 
{
    bool success = readSettingsTextFromFile(msr::airlib::Settings::Settings::getUserDirectoryFullPath("settings.json"), settingsText);
    return success;
}

std::string AirSimCarSettingsParser::getSimMode()
{
    msr::airlib::Settings& settings_json = msr::airlib::Settings::loadJSonString(settingsText_);
    return settings_json.getString("SimMode", "");
}

// mimics void ASimHUD::initializeSettings()
bool AirSimCarSettingsParser::initializeSettings()
{
    if (getSettingsText(settingsText_))
    {
        AirSimSettings::initializeSettings(settingsText_);

        // not sure where settings_json initialized in AirSimSettings::initializeSettings() is actually used
        msr::airlib::Settings& settings_json = msr::airlib::Settings::loadJSonString(settingsText_);
        std::string simmode_name = settings_json.getString("SimMode", "");
        std::cout << "simmode_name: " << simmode_name << std::endl; 

        AirSimSettings::singleton().load(std::bind(&AirSimCarSettingsParser::getSimMode, this));
    }
    else
    {
        return false;
    }
}