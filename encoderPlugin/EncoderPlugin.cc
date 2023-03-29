#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

	class EncoderPlugin : public ModelPlugin {

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&EncoderPlugin::OnUpdate, this));
			this->updateRate = common::Time(0, common::Time::SecToNano(0.05));  // 20 Hz

			// initialize the prevUpdateTime
			this->prevUpdateTime = common::Time::GetWallTime();
		}

	public: void OnUpdate() {
		if(common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate)
			return;

		this->prevUpdateTime = common::Time::GetWallTime();

		leftJoint = this->model->GetJoint("right_wheel_joint");
		rightJoint = this->model->GetJoint("left_wheel_joint");

		double leftVel = this->leftJoint->GetVelocity(0);
		double rightVel = this->rightJoint->GetVelocity(0);

		std::cout << "-------ENCODER PLUGIN-------" << '\n';
		std::cout << "Left wheel speed: " << leftVel << '\n';
		std::cout << "Right wheel speed: " << rightVel << '\n' << std::endl;
	}
	private: common::Time updateRate;
	private: common::Time prevUpdateTime;
	private: physics::ModelPtr model;
	private: physics::JointPtr leftJoint, rightJoint;
	private: event::ConnectionPtr updateConnection;
	};

	GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)
}
