#include "pch.h"
#include "PhysicsSandboxApp.h"

#include "SceneInspector.h"

using namespace std::chrono_literals;

namespace drb {

	// Helper Forward Decls
	namespace inspector {
		void DisplayRigidBodyInfo(physics::RigidBody const& rb);
		void DisplayFrameInfo(Float32 renderFPS, Int64 simTimeMicroSecs, Int64 frameCount);
	}


	PhysicsSandboxApp::~PhysicsSandboxApp() noexcept
	{
		inspector::Close();
	}


	bool PhysicsSandboxApp::Init(int argc, char* argv[])
	{
		using namespace physics;

		// Use default initialization
		bool result = Application::Init(argc, argv);
		if (not result) { return false; }

		// Disable V-Sync to test frame rates
		glfwSwapInterval(0);

		// Customize drawing mode and clear color
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

		// Build physics World
		mDummy.SetCollisionGeometry(std::make_shared<CollisionGeometry const>());

		auto geomA = std::make_shared<physics::CollisionGeometry>();
		geomA->AddCollider(
				Sphere{ .r = 2.0f },
				10.0f)
			.Bake();
		
		auto geomB = std::make_shared<physics::CollisionGeometry>();
		geomB->AddCollider(
			    Capsule{ 
					Segment{ .b = Vec3(0), .e = Vec3(0,2,0) }, 
					0.5f // radius
				},
				20.0f)
			.Bake();

		auto geomC = std::make_shared<physics::CollisionGeometry>();
		geomC->AddCollider(
				Convex::MakeTetrahedron(Vec3(0, 2, 0), Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(-1, 0, 0)),
				20.0f)
			 .AddCollider(Convex::MakeBox(Vec3(1.0f, 1.0f, 1.0f)))
			.Bake();


		mWorld.CreateRigidBody()
			.SetPosition(Vec3(0, 5, 0))
			.SetGravityScale(0.0f)
			.SetCollisionGeometry(geomC);

		mWorld.CreateRigidBody()
			.SetPosition(Vec3(5, 0, 0))
			.SetGravityScale(0.0f)
			.SetCollisionGeometry(geomA);
			
		
		mWorld.CreateRigidBody()
			.SetType(physics::RigidBody::Type::Kinematic)
			.SetPosition(Vec3(-5, 0, 0))
			.SetCollisionGeometry(geomC);
		
		
		mWorld.CreateRigidBody()
			.SetType(physics::RigidBody::Type::Kinematic)
			.SetPosition(Vec3(-10, 0, 0))
			.SetCollisionGeometry(geomB);
			

		mWorld.AddStaticCollider(Convex::MakeBox(Vec3(1.0f, 2.0f, 3.0f)));

		// Build BVH
		mWorld.Init();

		// Sync Renderer and Recorder with World
		mRenderer.Init(mWorld);
		mRecorder.Init(mWorld, 300); // record 300 frames (roughly 5 seconds)

		// Set Window values
		mWindow.SetTitle("Physics Sandbox");
		mWindow.SetDimensions(1280, 800);

		// Set Camera values
		mCam.SetPosition(Vec3(-16, 4, 30))
			.LookAt(Vec3(0));

		// Set Timer values
		mTimer.SetDeltaTimeSmoothing(0.7);
			//.SetFrameTimeCap(8333333ns); // cap at 120fps

		// Initialize Scene Inspector
		inspector::Init(mWindow);

		return result;
	}

	void PhysicsSandboxApp::Run()
	{
		Int64 totalSteps = 0, currentStep = 0;

		// Simulation variables
		Bool paused       = false;
		Bool singleStep   = false;
		int stepDirection = 0;  // +1 -> forward, -1 -> backward

		// Fixed delta time for physics
		FrameTimer::Duration constexpr fixedDeltaTime = 16666667ns; // update physics at 60fps
		Float32 constexpr                     fixedDT = static_cast<Float32>(fixedDeltaTime.count() * FrameTimer::durationTypeRatio);
		FrameTimer::Duration accumulatedTime = 0ns;
		
		// Variable delta time for rendering
		Float32 dt = 1.0f / 60.0f;

		while (not mWindow.ShouldClose())
		{
			// ------------------
			// Frame Timing Start
			// ------------------
			mTimer.FrameBegin();
			
			// -------------------------
			// Input & Application Logic
			// -------------------------
			GatherInputAndEvents();
			UpdateCamera(dt);
			MouseSelectRigidBody();

			// Simulation exit, pause, and stepping
			if (mEscape)							{ mWindow.Close(); }
			if (mNumKeys[1] && not mNumKeysPrev[1])	{ paused = not paused; }
			if (mNumKeys[2] && not mNumKeysPrev[2])	{ singleStep = not singleStep; }
			
			stepDirection = 0;
			if      (mNumKeys[0] && (not mNumKeysPrev[0] || not singleStep)) { stepDirection = 1; }
			else if (mNumKeys[9] && (not mNumKeysPrev[9] || not singleStep)) { stepDirection = -1; }

			
			// ----------------
			// Simulate Physics
			// ----------------
			FrameTimer::Duration simTime{};
			if (not paused)
			{
				accumulatedTime += mTimer.LastFrameTime();
				
				// DEBUG BEGIN
				auto simStartTime = FrameTimer::Now();
				// DEBUG END

				while (accumulatedTime >= fixedDeltaTime) 
				{
					mWorld.Step(fixedDT);

					mRecorder.Record();
					
					// Other work requiring fixed update
					mSelected->AddForce(Vec3(mArrows.x, mArrows.y, 0) * 100.0f);
					// ...

					accumulatedTime -= fixedDeltaTime;
					currentStep = ++totalSteps;
				}

				// DEBUG BEGIN
				simTime = FrameTimer::Now() - simStartTime;
				// DEBUG END

			}
			else // If we're paused, we can play our state forward or backward 
			{
				Int64 const stepsBehindCurrent = static_cast<Int64>(mRecorder.AdvanceSteps(stepDirection));

				currentStep = totalSteps - stepsBehindCurrent;
			}


			// ----
			// Draw
			// ----

			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// Draw scene
			Float32 const frameInterpolation = static_cast<Float32>(accumulatedTime.count() * FrameTimer::durationTypeRatio) / fixedDT;
			mRenderer
				.BeginDraw(mCam.ProjMatrix(), mCam.ViewMatrix(), mCam.GetPosition())
				.DrawRigidBodies(frameInterpolation)
				.HighlightOneRigidBody(*mSelected, frameInterpolation, ColorInfo{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.9f, 0.9f, 0.0f },
					.gloss = 0.82f,
					.opacity = 0.2f
					})
				.HighlightOneRigidBody(*mHovered, frameInterpolation, ColorInfo{
					.specular = { 0.02f, 0.02f, 0.02f },
					.diffuse = { 0.0f, 0.9f, 0.9f },
					.gloss = 0.82f,
					.opacity = 0.4f
					})
				.DrawStaticCollisionGeometry()
				.DrawContactManifolds()
				.DrawBVH()
				.EndDraw();
			
			
			// Draw inspector
			inspector::BeginDraw();
			inspector::DisplayRigidBodyInfo(*mSelected);
			inspector::DisplayFrameInfo( 1.0f / dt, std::chrono::duration_cast<std::chrono::microseconds>(simTime).count(), currentStep);
			inspector::FinishDraw();
			

			glfwSwapBuffers(mWindow.Get());
			

			// ----------------
			// Frame Timing End
			// ----------------
			
			mTimer.FrameEnd();
			dt = static_cast<Float32>( mTimer.DeltaTime() );
		}
	}

	// -------------------------------------------------------------------------
	// HELPERS
	// -------------------------------------------------------------------------

	void PhysicsSandboxApp::GatherInputAndEvents()
	{
		glfwPollEvents();

		// WINDOW
		int w{}, h{};
		glfwGetFramebufferSize(mWindow.Get(), &w, &h);
		mWindow.SetDimensions(w, h);
		glViewport(0, 0, w, h);

		// ESCAPE
		mEscape = false;
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
		{
			mEscape = true;
		}

		// WASD
		mWASD = Vec2(0);
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_W) == GLFW_PRESS) {
			mWASD.y += 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_S) == GLFW_PRESS) {
			mWASD.y -= 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_A) == GLFW_PRESS) {
			mWASD.x -= 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_D) == GLFW_PRESS) {
			mWASD.x += 1.0f;
		}
		if (mWASD.x != 0.0f && mWASD.y != 0.0f) {
			mWASD *= 0.7f;
		}

		// ARROWS
		mArrows = Vec2(0);
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_UP) == GLFW_PRESS) {
			mArrows.y += 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_DOWN) == GLFW_PRESS) {
			mArrows.y -= 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_LEFT) == GLFW_PRESS) {
			mArrows.x -= 1.0f;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_RIGHT) == GLFW_PRESS) {
			mArrows.x += 1.0f;
		}
		if (mArrows.x != 0.0f && mArrows.y != 0.0f) {
			mArrows *= 0.7f;
		}

		// NUMBER KEYS
		std::memcpy(mNumKeysPrev, mNumKeys, 10);
		std::memset(mNumKeys, 0, 10 * sizeof(Bool));
		{
#define HANDLE_NUM_KEY(num) if (glfwGetKey(mWindow.Get(), GLFW_KEY_##num) == GLFW_PRESS) { mNumKeys[num] = true; }

			HANDLE_NUM_KEY(1)
			HANDLE_NUM_KEY(2)
			HANDLE_NUM_KEY(3)
			HANDLE_NUM_KEY(4)
			HANDLE_NUM_KEY(5)
			HANDLE_NUM_KEY(6)
			HANDLE_NUM_KEY(7)
			HANDLE_NUM_KEY(8)
			HANDLE_NUM_KEY(9)
			HANDLE_NUM_KEY(0)

#undef HANDLE_NUM_KEY
		}

		// MOUSE
		double mouseX{}, mouseY{};
		glfwGetCursorPos(mWindow.Get(), &mouseX, &mouseY);
		mMouseDelta.x = static_cast<float>(mouseX) - mMouse.x;
		mMouseDelta.y = static_cast<float>(mouseY) - mMouse.y;
		mMouse.x	  = static_cast<float>(mouseX);
		mMouse.y	  = static_cast<float>(mouseY);
		mMouseLeft    = glfwGetMouseButton(mWindow.Get(), GLFW_MOUSE_BUTTON_LEFT);
		mMouseRight   = glfwGetMouseButton(mWindow.Get(), GLFW_MOUSE_BUTTON_RIGHT);
	}

	void PhysicsSandboxApp::UpdateCamera(Float32 dt)
	{
		static constexpr Float32 speed = 10.0f;
		static constexpr Float32 rotSpeed = 0.2f;

		Vec3    const fwd = mCam.GetForward(),
					  right = mCam.GetRight(),
					  pos = mCam.GetPosition(),
					  up = mCam.GetUp();

		// Move camera using WASD
		mCam.SetPosition(pos + ((fwd * speed) * mWASD.y + (right * speed) * mWASD.x) * dt);

		// Rotate forward direction using mouse delta
		if (mMouseRight) {
			mCam.SetForwardDirection(fwd + (mMouseDelta.x * (right * rotSpeed) + -mMouseDelta.y * (up * rotSpeed)) * dt);
		}

		// Update viewport
		mCam.SetViewportDimensions(mWindow.WidthF(), mWindow.HeightF());
	}

	void PhysicsSandboxApp::MouseSelectRigidBody()
	{
		Mat4 const invProj = glm::inverse(mCam.ProjMatrix());
		Mat4 const invView = glm::inverse(mCam.ViewMatrix());

		// Convert mouse screen coords to homogeneous clip coords
		Vec4 const rayClip{
			2.0f * mMouse.x / mWindow.WidthF() - 1.0f,
			1.0f - (2.0f * mMouse.y) / mWindow.HeightF(),
			-1.0f,
			1.0f 
		};

		// Convert clip space to camera space
		Vec4 rayCam = invProj * rayClip;
		rayCam.z = -1.0f;
		rayCam.w = 0.0f;

		// Convert ray in camera space to world space
		Vec3 const rayWorld = Normalize( Vec3(invView * rayCam) );

		physics::Ray const r{ .p = mCam.GetPosition(), .d = rayWorld};

		// Check which rigidbody, if any, the ray hit
		auto const [proxy, result] = mWorld.RayCastQuery(r);

		if (proxy.rb) {
			mHovered = proxy.rb;
		}
		else {
			mHovered = &mDummy;
		}

		if (mMouseLeft) {
			mSelected = mHovered;
		}
	}

	namespace inspector {
		
		void DisplayFrameInfo(Float32 renderFPS, Int64 simTime, Int64 frameCount)
		{
			static Uint64 counter = 0;
			static Float32 avgFPS = 0.0f;
			static Float32 totalFPS = 0.0f;
			static Float32 avgSimTime = 0;
			static Int64 totalSimTime = 0;

			if (counter++ % 120 == 0) {
				avgFPS = (1.0f / 120.0f) * totalFPS;
				totalFPS = 0.0f;
				avgSimTime = (1.0f / 120.0f) * totalSimTime;
				totalSimTime = 0;
			}
			totalFPS += renderFPS;
			totalSimTime += simTime;

			ImGui::Begin("Simulation Info");
		
			ImGui::Text("Render FPS: %.0f", avgFPS);
			ImGui::Text("Sim Time per Frame: %.0f microseconds", avgSimTime);
			ImGui::Text("Sim Step: %d", frameCount);

			ImGui::End();
		}

		void DisplayRigidBodyInfo(physics::RigidBody const& rb)
		{
			physics::RigidBodyState rbState{};
			rbState.CopyFrom(rb);
			
			ImGui::Begin("RigidBody Viewer");

			ImGui::DragFloat3("Position", glm::value_ptr(rbState.position));

			Vec3 eulerAngles = glm::eulerAngles(rbState.orientation);
			ImGui::DragFloat3("Orientation (Euler)", glm::value_ptr(eulerAngles));
			
			ImGui::DragFloat3("Linear Velocity", glm::value_ptr(rbState.linearVelocity));
			ImGui::DragFloat3("Angular Velocity", glm::value_ptr(rbState.angularVelocity));

			ImGui::End();
		}

	}
}