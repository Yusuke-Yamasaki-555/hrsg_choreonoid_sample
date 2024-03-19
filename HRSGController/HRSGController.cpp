// ref: https://choreonoid.org/ja/manuals/latest/simulation/howto-implement-controller.html
  // この公式サンプルコード（SR1MinimumController)のコピー、および改変。
  // ライセンスを確認しておくこと。
// ref: https://github.com/open-rdc/biped-control-lib/blob/master/choreonoid/simple_controller/PDController/PDController.cpp



#include <cnoid/SimpleController>

#include <iostream>
#include <vector>


using namespace cnoid;


const double GAIN_P[] = {  // 各関節のPゲイン
  8000, 8000, 8000, 8000, 8000, 8000,
  3000, 3000, 3000, 3000, 3000, 3000, 3000,
  8000, 8000, 8000, 8000, 8000, 8000,
  3000, 3000, 3000, 3000, 3000, 3000, 3000,
  8000, 8000, 8000
};

const double GAIN_D[] = {  // 各関節のDゲイン
  100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 100, 100, 100,
  100, 100, 100
};

// /*
const double Q_INIT[] = {  // 各関節の初期角度 (急に角度を変えると、吹っ飛ぶ)
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0,
  0, 0, 0
};  // */


class HRSGController : public SimpleController
{
  public:
    virtual bool initialize(SimpleControllerIO* io) override {  // 初期化
      body_ = io->body();  // ロボット情報
      dt_ = io->timeStep();  // 単位時間 == 1 [ms] == 0.001 [s]
      // std::cout << dt_ << std::endl;  // DEBUG

      for(int i = 0; i < body_->numJoints(); i++) {
        Link* joint = body_->joint(i);
        // joint->setActuationMode(Link::JointTorque);  // Torque Control
        joint->setActuationMode(Link::JointAngle);  // Position Control
        // joint->setActuationMode(Link::JointAngle | Link::JointVelocity);  // Position & Velocity Control
        io->enableIO(joint);
        // q_ref_.push_back(joint->q());  // Torque Control
      }

      // q_old_ = q_ref_;  // Torque Control

      // /* Position Control
      for(int i = 0; i < body_->numJoints(); i++) {
        body_->joint(i)->q_target() = Q_INIT[i];  // set position_q  (if velocity control, q_target -> dq_target)
      }
      // */

      return true;
    }

    virtual bool control() override {  // 周期実行
      control_count_ = (control_count_+1) % 10; // 10 [ms]に合わせてカウント。0 ~ 9カウント, 0 -> 10 [ms]周期。
     
      /* Torque Control
      for(int i = 0; i < body_->numJoints(); i++) {
        Link* joint = body_->joint(i);
        double q = joint->q();  // 現在角度の取得
        double dq = (q - q_old_[i]) / dt_;  // １ｓ当たりの関節角度変化
        double u = (q_ref_[i] - q) * GAIN_P[i] + (0.0 - dq) * GAIN_D[i];  // 入力（トルク）の算出
        q_old_[i] = q;  // 記憶
        joint->u() = u;  // 入力の適用
      }
      */

      // /* Position Control
      if(control_count_ == 0) {
        step++;
        // Update Joint Position & Velocity
        Trajectory2Joints();

        for(int i = 0; i < body_->numJoints(); i++) {
          body_->joint(i)->q_target() = q_target_[i];  // set position_q  (if velocity control, q_target -> dq_target)
          body_->joint(i)->dq_target() = dq_target_[i];  // set velocity_q
        }
        
        // std::cout << control_time_ << std::endl;
      }
      // */

      control_time_ += dt_;
      // std::cout << control_time_ << std::endl;
      return true;
    }

  private:
    void Trajectory2Joints(void) {
      if(step <= 500) {
        q_target_[1] -= 0.001;
        q_target_[3] += 0.002;
        q_target_[4] -= 0.001;
        q_target_[14] -= 0.001;
        q_target_[16] += 0.002;
        q_target_[17] -= 0.001;
      }
    }

    BodyPtr body_;
    double dt_;  // 0.001 [s] == 1 [ms]
    int control_count_ = -1;  // ChoreonoidのTimeStepと制御周期の合わせに利用。
    float control_time_ = 0;
    int step = 0;
    
    /* Torque Control
    std::vector<double> q_ref_;
    std::vector<double> q_old_;
    */

    // /* Position Control
    double q_target_[29] = {0};
    double dq_target_[29] = {
      1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 
      1, 1, 1, 1, 1, 1, 
      1, 1, 1, 1, 1, 1, 1, 
      1, 1, 1
    };
    // */

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HRSGController)
