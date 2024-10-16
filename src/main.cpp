#include <aris.hpp>

#include "plan.hpp"
#include "robot.hpp"


const std::filesystem::path path = "C:/Users/11051/Source/repos/JaGuarPENG/A10_Single/cs.xml ";

int main(int argc, char *argv[])
{

    auto& cs = aris::server::ControlServer::instance();

    aris::core::fromXmlFile(cs, path);


    cs.init();
    
    cs.open();

    cs.start();
    
    cs.runCmdLine();

    return 0;
}
