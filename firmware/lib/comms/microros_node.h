#pragma once

#include <Arduino.h>

namespace comms
{
    // runs entirely inside comms_task on core 0, isolated from the control loop.
    // if the agent disconnects, the node attempts to reconnect without
    // blocking motor control and safety tasks on core 1.
    // ============================================================================
    class MicroRosNode
    {
    public:
        // initialize the serial transport; the session is created in spinOnce
        void begin();

        // called periodically from comms_task. Handles the agent connection,
        // executor spin and publishing. Non-blocking with short timeouts.
        void spinOnce();

    private:
        enum class AgentState
        {
            WAITING,     // no agent, ping until one answers
            AVAILABLE,   // agent answered, create the session
            CONNECTED,   // spin and publish, ping to notice a lost agent
            DISCONNECTED // agent lost, tear down and wait again
        };

        bool createEntities();
        void destroyEntities();
        void publishJointStates();
        void publishImu();
        void publishSteering();

        AgentState state_ = AgentState::WAITING;
        uint32_t last_publish_ms_ = 0;
        uint32_t last_ping_ms_ = 0;
    };
}
