#ifndef MOVE3DMAIN_HPP
#define MOVE3DMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

using namespace OpenRAVE;

class Move3dProblem : public ModuleBase
{
public:
    Move3dProblem(EnvironmentBasePtr penv);
    virtual ~Move3dProblem();
    void Destroy();

    virtual int main( const std::string& args );
    virtual bool SendCommand( std::ostream& sout, std::istream& sinput );

    bool NumBodies( std::ostream& sout, std::istream& sinput );

    bool InitMove3dEnv();
    bool LoadConfigFile( std::istream& sinput );
    bool RunRRT();
    bool RunStomp();

private:
    std::string strRobotName_; ///< name of the active robot
    RobotBasePtr robot_;
    EnvironmentBasePtr env_;
};

#endif // MOVE3DMAIN_HPP
