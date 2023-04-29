#pragma once
#include "Application.h"

#include "PhysicsWorld.h"
#include "PhysicsDebugDrawing.h"
#include "WorldStateRecorder.h"
#include "Camera.h"
#include "FrameTimer.h"

namespace drb {
    class PhysicsSandboxApp final : public Application
    {
    private:
        // Managers
        std::vector<physics::World> mWorlds{};
        std::vector<physics::DebugRenderer> mRenderers{};
        std::vector<physics::WorldStateRecorder> mRecorders{};
        Camera mCam{};
        FrameTimer mTimer{};

        // Input
        glm::vec2 mMouse{};
        glm::vec2 mMouseDelta{};
        Bool mMouseLeft = false, mMouseRight = false;
        glm::vec2 mWASD{};
        Vec2 mArrows{};
        Bool mNumKeys[10] = {}, mNumKeysPrev[10] = {};
        Bool mEscape = false;

        // Inspection/User Control
        physics::RigidBody mDummy{};
        physics::RigidBody* mSelected = &mDummy;
        physics::RigidBody* mHovered = &mDummy;

    public:
        virtual ~PhysicsSandboxApp() noexcept override;

        virtual bool Init(int argc, char* argv[]) override;

        virtual void Run() override;

    private:
        void GatherInputAndEvents();

        void UpdateCamera(Float32 dt);

        void MouseSelectRigidBody();
    };
}

