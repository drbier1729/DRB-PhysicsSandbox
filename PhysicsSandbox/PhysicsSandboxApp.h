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
        physics::World mWorld{ 1024, 4096, 20 };
        physics::DebugRenderer mRenderer{};
        physics::WorldStateRecorder mRecorder{};
        Camera mCam{};
        FrameTimer mTimer{};

        // Input
        Vec2 mMouse{};
        Vec2 mMouseDelta{};
        Bool mMouseLeft = false, mMouseRight = false;
        Vec2 mWASD{};
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

