#include "pch.h"
#include "PhysicsSandboxApp.h"

#include "SceneInspector.h"

#include "DRBAssert.h"

using namespace std::chrono_literals;


namespace drb {
	enum Scene
	{
		BOX_STACKS = 0,
		NONCONVEX = 1,
		TALL_STACK = 2,
		SPH_CAP = 3,
		COUNT
	};

	static bool drawBVH = false;
	static bool drawManifolds = false;
	static Scene activeWorld = Scene::NONCONVEX;
	static const char* simmingMsg = "SIMULATING";
	static const char* historyMsg = "REPLAY";
	static const char** currMsg = &simmingMsg;

	// Helper Forward Decls
	namespace inspector {
		void DisplayRigidBodyInfo(physics::RigidBody const& rb);
		void DisplayFrameInfo(float renderFPS, Int64 simTimeMicroSecs, Int64 frameCount);
		void ChooseScene();
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
		mWorlds.reserve(Scene::COUNT);
		for (auto i = 0; i < Scene::COUNT; ++i) { 
			mWorlds.emplace_back(1024, 4096, 10); 
		}
		mRenderers.resize(Scene::COUNT);
		mRecorders.resize(Scene::COUNT);

		auto geomA = std::make_shared<CollisionGeometry>();
		geomA->AddCollider(
			Convex::MakeTetrahedron(Vec3(0, 0, 0), Vec3(1, -2, 0), Vec3(0, -2, 1), Vec3(-1, -2, 0)),
			20.0_r)
			.AddCollider(
				Convex::MakeTetrahedron(Vec3(0, 0, 0), Vec3(1, 2, 0), Vec3(0, 2, 1), Vec3(-1, 2, 0)),
				20.0_r)
			.Bake();

		auto geomB = std::make_shared<CollisionGeometry>();
		geomB->AddCollider(
			Convex::MakeBox(Vec3(1, 1, 1)),
			20.0_r)
			.Bake();

		auto geomC = std::make_shared<CollisionGeometry>();
		geomC->AddCollider(Capsule(Segment{ .b = Vec3(0), .e = Vec3(0,2,0) }, 0.5_r), 100.0_r)
			.Bake();
		
		auto geomD = std::make_shared<CollisionGeometry>();
		geomD->AddCollider(Sphere{.c = Vec3(0), .r = 2.0_r}, 100.0_r)
			.Bake();

		CollisionGeometry staticGeoA{};
		staticGeoA.AddCollider(
			Convex::MakeQuad(100, 100),
			0.0)
			.Bake();

		// Sphere and Caps scene
		for (auto i = 1; i < 3; ++i) {
			mWorlds[Scene::SPH_CAP].CreateRigidBody()
				.SetCollisionGeometry(geomC)
				.SetPosition(Vec3(5 * i, 5, 0))
				.SetGravityScale(0.0_r)
				.SetFriction(0.6_r)
				.SetRestitution(0.5_r);
			
			mWorlds[Scene::SPH_CAP].CreateRigidBody()
				.SetCollisionGeometry(geomD)
				.SetPosition(Vec3(-5 * i, 5, 0))
				.SetGravityScale(0.0_r)
				.SetFriction(0.6_r)
				.SetRestitution(0.5_r);
		}

		// Non convex
		mWorlds[Scene::NONCONVEX].CreateRigidBody()
			.SetCollisionGeometry(geomA)
			.SetPosition(Vec3(5, 5, 0))
			.SetGravityScale(1.0_r)
			.SetFriction(0.6_r)
			.SetRestitution(0.3_r);
		
		mWorlds[Scene::NONCONVEX].CreateRigidBody()
			.SetCollisionGeometry(geomA)
			.SetPosition(Vec3(-5, 5, 0))
			.SetGravityScale(1.0_r)
			.SetFriction(0.6_r)
			.SetRestitution(0.3_r);

		mWorlds[Scene::NONCONVEX].CreateRigidBody()
			.SetCollisionGeometry(geomB)
			.SetPosition(Vec3(0, 3, 0))
			.SetGravityScale(1.0_r)
			.SetFriction(0.5_r);
		mWorlds[Scene::NONCONVEX].CreateStaticCollisionGeometry(staticGeoA);

		// Stacks
		for (Int32 i = 0; i < 5; ++i) {
			for (Int32 k = 0; k < 5; ++k) {
				for (Int32 j = 0; j < 5; ++j) {
					mWorlds[Scene::BOX_STACKS].CreateRigidBody()
						.SetCollisionGeometry(geomB)
						.SetPosition(Vec3(3*(i+1), 3 * (j + 1), 3*(k+1)))
						.SetGravityScale(1.0_r)
						.SetFriction(1.0_r)
						.SetMass(1.0_r);
				}
			}
		}
		mWorlds[Scene::BOX_STACKS].CreateStaticCollisionGeometry(staticGeoA);

		// Tall stack
		for (Int32 j = 0; j < 30; ++j) {
			mWorlds[Scene::TALL_STACK].CreateRigidBody()
				.SetCollisionGeometry(geomB)
				.SetPosition(Vec3(0, 2.1 * (j + 1), 0))
				.SetGravityScale(1.0_r)
				.SetFriction(1.0_r)
				.SetMass((j + 1) * 20.0_r);
		}
		mWorlds[Scene::TALL_STACK].CreateStaticCollisionGeometry(staticGeoA);

		// Build BVHs
		for (auto&& w : mWorlds) {
			w.Init();
		}

		// Sync Renderers and Recorders with Worlds
		for (Int32 i = 0;  auto && r : mRenderers) {
			r.Init(mWorlds[i++]);
		}
		for (Int32 i = 0;  auto && r : mRecorders) {
			r.Init(mWorlds[i++], 300); // record 300 frames (roughly 5 seconds)
		}

		// Set Window values
		mWindow.SetTitle("Physics Sandbox");
		mWindow.SetDimensions(1280, 800);

		// Set Camera values
		mCam.SetPosition(Vec3(-16, 4, 30))
			.LookAt(Vec3(0));

		// Set Timer values
		mTimer.SetDeltaTimeSmoothing(0.7);
			//.SetFrameTimeCap(8333333ns); // cap at 120ps

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
		FrameTimer::Duration constexpr fixedDeltaTime = 16666667ns; // update physics at 60ps
		Real constexpr                 fixedDT = static_cast<Real>(fixedDeltaTime.count() * FrameTimer::durationTypeRatio);
		FrameTimer::Duration           accumulatedTime = 0ns;
		
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
				currMsg = &simmingMsg;

				accumulatedTime += mTimer.LastFrameTime();
				
				// DEBUG BEGIN
				auto simStartTime = FrameTimer::Now();
				// DEBUG END

				while (accumulatedTime >= fixedDeltaTime) 
				{
					mWorlds[activeWorld].Step(fixedDT);

					mRecorders[activeWorld].Record();
					
					// Other work requiring fixed update
					mSelected->AddForce(Vec3(mArrows.x, mArrows.y, 0) * 2000.0_r);
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
				currMsg = &historyMsg;

				Int64 const stepsBehindCurrent = static_cast<Int64>(mRecorders[activeWorld].AdvanceSteps(stepDirection));
				currentStep = totalSteps - stepsBehindCurrent;
				
				// If we're up to the current step, continue simulating forward
				if (stepsBehindCurrent == 0 && stepDirection >= 0) 
				{
					currMsg = &simmingMsg;
					for (auto i = 0; i < stepDirection; ++i) {
						mWorlds[activeWorld].Step(fixedDT);

						mRecorders[activeWorld].Record();

						// Other work requiring fixed update
						mSelected->AddForce(Vec3(mArrows.x, mArrows.y, 0) * 2000.0_r);
						// ...

						currentStep = ++totalSteps;
					}
				}
			}


			// ----
			// Draw
			// ----

			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// Draw scene
			float const frameInterpolation = static_cast<float>(accumulatedTime.count() * FrameTimer::durationTypeRatio) / static_cast<float>(fixedDT);
			mRenderers[activeWorld]
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
				.DrawStaticCollisionGeometry();
			if (drawBVH) { mRenderers[activeWorld].DrawBVH(); }
			if (drawManifolds) { mRenderers[activeWorld].DrawContactManifolds(); }
			
			mRenderers[activeWorld].EndDraw();
			
			
			// Draw inspector
			inspector::BeginDraw();
			inspector::DisplayRigidBodyInfo(*mSelected);
			inspector::DisplayFrameInfo( 1.0f / dt, std::chrono::duration_cast<std::chrono::microseconds>(simTime).count(), currentStep);
			inspector::ChooseScene();
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

		// WASD -- camera controls
		mWASD = glm::vec2(0);
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_W) == GLFW_PRESS) {
			mWASD.y += 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_S) == GLFW_PRESS) {
			mWASD.y -= 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_A) == GLFW_PRESS) {
			mWASD.x -= 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_D) == GLFW_PRESS) {
			mWASD.x += 1.0_r;
		}
		if (mWASD.x != 0.0_r && mWASD.y != 0.0_r) {
			mWASD *= 0.7_r;
		}

		// ARROWS -- rigidbody controls
		mArrows = Vec2(0);
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_UP) == GLFW_PRESS) {
			mArrows.y += 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_DOWN) == GLFW_PRESS) {
			mArrows.y -= 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_LEFT) == GLFW_PRESS) {
			mArrows.x -= 1.0_r;
		}
		if (glfwGetKey(mWindow.Get(), GLFW_KEY_RIGHT) == GLFW_PRESS) {
			mArrows.x += 1.0_r;
		}
		if (mArrows.x != 0.0_r && mArrows.y != 0.0_r) {
			mArrows *= 0.7_r;
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
		mMouseDelta.x = static_cast<Float32>(mouseX) - mMouse.x;
		mMouseDelta.y = static_cast<Float32>(mouseY) - mMouse.y;
		mMouse.x	  = static_cast<Float32>(mouseX);
		mMouse.y	  = static_cast<Float32>(mouseY);
		mMouseLeft    = glfwGetMouseButton(mWindow.Get(), GLFW_MOUSE_BUTTON_LEFT);
		mMouseRight   = glfwGetMouseButton(mWindow.Get(), GLFW_MOUSE_BUTTON_RIGHT);
	}

	void PhysicsSandboxApp::UpdateCamera(Float32 dt)
	{
		static constexpr Float32 speed = 10.0f;
		static constexpr Float32 rotSpeed = 0.2f;

		glm::vec3 const fwd = mCam.GetForward(),
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
		glm::mat4 const invProj = glm::inverse(mCam.ProjMatrix());
		glm::mat4 const invView = glm::inverse(mCam.ViewMatrix());

		// Convert mouse screen coords to homogeneous clip coords
		glm::vec4 const rayClip{
			2.0f * mMouse.x / mWindow.WidthF() - 1.0f,
			1.0f - (2.0f * mMouse.y) / mWindow.HeightF(),
			-1.0f,
			1.0f 
		};

		// Convert clip space to camera space
		glm::vec4 rayCam = invProj * rayClip;
		rayCam.z = -1.0f;
		rayCam.w = 0.0f;

		// Convert ray in camera space to world space
		Vec3 const rayWorld = Normalize( Vec3(invView * rayCam) );

		physics::Ray const r{ .p = Vec3(mCam.GetPosition()), .d = rayWorld};

		// Check which rigidbody, if any, the ray hit
		auto const [proxy, result] = mWorlds[activeWorld].RayCastQuery(r);

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
		
		void DisplayFrameInfo(float renderFPS, Int64 simTime, Int64 frameCount)
		{
			static Uint64 counter = 0;
			static float avgFPS = 0.0;
			static float totalFPS = 0.0;
			static float avgSimTime = 0;
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
		
			ImGui::Text("Render FPS: %f", avgFPS);
			ImGui::Text("Sim Time per Frame: %f microseconds", avgSimTime);
			ImGui::Text("Sim Step: %d", frameCount);
			ImGui::Text("STATUS: %s", *currMsg);

			if (ImGui::Button("Draw Bounding Volume Hierarchy")) { drawBVH = not drawBVH; }
			if (ImGui::Button("Draw Contact Manifolds")) { drawManifolds = not drawManifolds; }

			ImGui::End();
		}
		
		void ChooseScene()
		{
			ImGui::Begin("Select Scene");

			if (ImGui::Button("Box Stacks"))       { activeWorld = Scene::BOX_STACKS;}
			if (ImGui::Button("Tall Stack"))       { activeWorld = Scene::TALL_STACK;}
			if (ImGui::Button("Non-Convex"))       { activeWorld = Scene::NONCONVEX;}
			if (ImGui::Button("Spheres/Capsules")) { activeWorld = Scene::SPH_CAP; }

			ImGui::End();
		}

		void DisplayRigidBodyInfo(physics::RigidBody const& rb)
		{
			auto imguiDataType = ImGuiDataType_Float;
			if constexpr (sizeof(Real) == sizeof(double)) 
			{
				imguiDataType = ImGuiDataType_Double;
			}

			physics::RigidBodyState rbState{};
			rbState.CopyFrom(rb);

			ImGui::Begin("RigidBody Viewer");
			ImGui::DragScalarN("Position", imguiDataType, glm::value_ptr(rbState.position), 3);

			Vec3 eulerAngles = glm::eulerAngles(rbState.orientation);
			ImGui::DragScalarN("Orientation (Euler)", imguiDataType, glm::value_ptr(eulerAngles), 3);

			ImGui::DragScalarN("Linear Velocity", imguiDataType, glm::value_ptr(rbState.linearVelocity), 3);
			ImGui::DragScalarN("Angular Velocity", imguiDataType, glm::value_ptr(rbState.angularVelocity), 3);

			ImGui::End();
		}

	}
}