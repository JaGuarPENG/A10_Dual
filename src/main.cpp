#include <aris.hpp>

#include "plan.hpp"
#include "robot.hpp"


const std::filesystem::path path = "C:/Users/11051/source/repos/A10_Dual/cs.xml ";

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
