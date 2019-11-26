```cpp
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main()
{
    el::Configurations defaultConf;
    defaultConf.setToDefault();
    // Values are always std::string
    defaultConf.set(el::Level::Info,
        el::ConfigurationType::Format, "%datetime %level %msg");
    // default logger uses default configurations
    el::Loggers::reconfigureLogger("default", defaultConf);
    // To set GLOBAL configurations you may use
    defaultConf.setGlobally(
            el::ConfigurationType::Format, "%date %msg");
    el::Loggers::reconfigureLogger("default", defaultConf);
    
    LOG(INFO) << "loading config file: " << fileNameWithPath;

    return 0;
}
```