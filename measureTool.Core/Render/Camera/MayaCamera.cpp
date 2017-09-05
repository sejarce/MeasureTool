#include "MayaCamera.h"

#include "Ogre.h"

class	MayaCamera::Imp
{
public:

	Ogre::Camera*		Camera_{};
	Ogre::RenderTarget*	RT_{};
	Ogre::SceneManager*	Smgr_{};
	Ogre::SceneNode*	PositionNode_{};
	Ogre::SceneNode*	TargetNode_{};
	Ogre::SceneNode*	Yaw_{};
	Ogre::SceneNode*	Pitch_{};
	float 				RotationSpeed_{.25f};
	bool				MouseRot_{false};
	bool				MouseDrag_{ false };
	OIS::MouseState		LastMouseState_;

public:

	void	UpdateData(const Ogre::Vector3& target, const Ogre::Vector3& pos)
	{
		//视线方向Z
		auto rotateTo = pos - target;

		TargetNode_->setPosition(target);
		PositionNode_->setPosition(0, 0, rotateTo.normalise());

		auto pitch = Ogre::Quaternion::IDENTITY;
		auto yaw = Ogre::Quaternion::IDENTITY;

		auto step1 = rotateTo;
		step1.y = 0;
		auto length = step1.normalise();

		if ( length > 0.f )
		{
			yaw = step1.getRotationTo(Ogre::Vector3::UNIT_Z);
		}

		auto step2 =  yaw * rotateTo;

		pitch = step2.getRotationTo(Ogre::Vector3::UNIT_Z);

		Pitch_->setOrientation(pitch.Inverse());
		Yaw_->setOrientation(yaw.Inverse());
	}
};

MayaCamera::MayaCamera( Ogre::Camera *camera, Ogre::RenderTarget* rt ):ICameraFrameListener(camera), ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Camera_ = camera;
	imp_.Smgr_ = camera->getSceneManager();
	imp_.RT_ = rt;

	imp_.TargetNode_ = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
	imp_.TargetNode_->setName("MayaCamera::TargetNode_");
	imp_.Yaw_ = imp_.TargetNode_->createChildSceneNode();
	imp_.Yaw_->setName("MayaCamera::Yaw_");
	imp_.Pitch_ = imp_.Yaw_->createChildSceneNode();
	imp_.Pitch_->setName("MayaCamera::Pitch_");
	imp_.PositionNode_ = imp_.Pitch_->createChildSceneNode();
	imp_.PositionNode_->setName("MayaCamera::PositionNode_");
	imp_.PositionNode_->setPosition(0, 0, 1.f);

	imp_.Camera_->setNearClipDistance(.1f);
	imp_.Camera_->setAutoAspectRatio(true);
	imp_.Camera_->setFOVy(Ogre::Degree(60));
	imp_.Camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
	imp_.Camera_->setFixedYawAxis(true);
	imp_.Camera_->setOrientation(Ogre::Quaternion::IDENTITY);

	imp_.Camera_->detachFromParent();
	imp_.PositionNode_->attachObject(camera);
}

MayaCamera::~MayaCamera()
{
	auto& imp_ = *ImpUPtr_;

	Unload();

	imp_.Camera_->detachFromParent();
	imp_.Smgr_->getRootSceneNode()->removeAndDestroyChild(imp_.TargetNode_);
}

void MayaCamera::SetPosAndTarget( const Ogre::Vector3 &pos, const Ogre::Vector3 &target )
{
	auto& imp_ = *ImpUPtr_;

	imp_.UpdateData(target, pos);
}

void MayaCamera::SetPosition( const Ogre::Vector3 &pos )
{
	auto& imp_ = *ImpUPtr_;

	imp_.UpdateData(imp_.TargetNode_->getPosition(), pos);
}

Ogre::Vector3 MayaCamera::GetPosition() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PositionNode_->_getDerivedPosition();
}

void MayaCamera::SetTarget( const Ogre::Vector3 &target )
{
	auto& imp_ = *ImpUPtr_;

	imp_.UpdateData(target, imp_.PositionNode_->_getDerivedPosition());
}

Ogre::Vector3 MayaCamera::GetTarget() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.TargetNode_->getPosition();
}

Ogre::Vector3 MayaCamera::GetDir() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Yaw_->getOrientation() * imp_.Pitch_->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
}

void MayaCamera::SetFocusLength( float length )
{
	auto& imp_ = *ImpUPtr_;

	imp_.PositionNode_->setPosition(0, 0, length);
}

float MayaCamera::GetFocusLength() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PositionNode_->getPosition().z;
}

void MayaCamera::SetRotationSpeed( float rSpeed )
{
	auto& imp_ = *ImpUPtr_;

	imp_.RotationSpeed_ = rSpeed;
}

float MayaCamera::GetRotationSpeed() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.RotationSpeed_;
}

void MayaCamera::_FrameStart( const Ogre::FrameEvent &fevt )
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecoder = GetSysEventRecorder();

	if ( !imp_.MouseRot_ && evtRecoder.HasMousePressed(OIS::MB_Right) )
	{
		imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();

		imp_.MouseRot_ = true;
		imp_.MouseDrag_ = false;
	}
	else if ( imp_.MouseRot_ && evtRecoder.HasMouseReleased(OIS::MB_Right) )
	{
		imp_.MouseRot_ = false;
	}

	if ( !imp_.MouseRot_ && !imp_.MouseDrag_ && evtRecoder.HasMousePressed(OIS::MB_Middle) )
	{
		imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();
		imp_.MouseDrag_ = true;
	}
	else if ( imp_.MouseDrag_ && evtRecoder.HasMouseReleased(OIS::MB_Middle) )
	{
		imp_.MouseDrag_ = false;
	}

	if ( evtRecoder.HasWheelRoll() )
	{
		auto needUpdate = false;

		auto z = evtRecoder.GetMouseState().Z.rel;
		
		auto target = GetTarget();
		auto dir = GetPosition() - target;
		auto focusLen = dir.normalise();

		if ( z > 0 )
		{
			if ( focusLen > 0.1f )
			{
				focusLen *= 0.9f;
				focusLen = std::max(focusLen, .1f);
				needUpdate = true;
			}
		}
		else
		{
			if ( focusLen < 5.f )
			{
				focusLen *= 1.2f;
				focusLen = std::min(focusLen, 5.f);
				needUpdate = true;
			}
		}

		if ( needUpdate )
		{
			imp_.UpdateData(target, target + dir * focusLen);
		}
	}

	if ( imp_.MouseRot_ )
	{
		auto offset = SysEventRecorder::CachedMouseState() - imp_.LastMouseState_;

		if ( 0 != offset.X.rel || 0 != offset.Y.rel )
		{
			auto yaw = Ogre::Degree(-offset.X.rel * imp_.RotationSpeed_) + imp_.Yaw_->getOrientation().getYaw();
			Ogre::Quaternion quaYaw;
			quaYaw.FromAngleAxis(yaw, Ogre::Vector3::UNIT_Y);
			imp_.Yaw_->setOrientation(quaYaw);

			auto pitch = Ogre::Degree(-offset.Y.rel * imp_.RotationSpeed_) + imp_.Pitch_->getOrientation().getPitch();
			Ogre::Quaternion quaPitch;
			quaPitch.FromAngleAxis(pitch, Ogre::Vector3::UNIT_X);

			auto pitchAngle = quaPitch.getPitch();
			auto tooSmall = std::numeric_limits<Ogre::Real>::epsilon();

			if ( pitchAngle > Ogre::Radian(Ogre::Degree(90 - tooSmall)) )
			{
				quaPitch.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
			}
			else if ( pitchAngle < Ogre::Radian(Ogre::Degree(-90 + tooSmall)) )
			{
				quaPitch.FromAngleAxis(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
			}

			imp_.Pitch_->setOrientation(quaPitch);

			imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();
		}
	}

	if ( imp_.MouseDrag_ )
	{
		auto offset = SysEventRecorder::CachedMouseState() - imp_.LastMouseState_;

		if ( 0 != offset.X.rel || 0 != offset.Y.rel )
		{
			auto target = imp_.TargetNode_->getPosition();
			auto pos = imp_.PositionNode_->_getDerivedPosition();
			auto posToTar = target - pos;

			auto focusLen = posToTar.normalise();

			Ogre::Ray r1;
			{
				auto x = static_cast<float>( imp_.LastMouseState_.X.rel ) / imp_.RT_->getWidth();
				auto y = static_cast<float>( imp_.LastMouseState_.Y.rel ) / imp_.RT_->getHeight();

				r1 = imp_.Camera_->getCameraToViewportRay(x, y);
			}

			auto v1 = r1.getPoint(focusLen);
			
			Ogre::Ray r2;
			{
				auto x = static_cast<float>( SysEventRecorder::CachedMouseState().X.rel ) / imp_.RT_->getWidth();
				auto y = static_cast<float>( SysEventRecorder::CachedMouseState().Y.rel ) / imp_.RT_->getHeight();

				r2 = imp_.Camera_->getCameraToViewportRay(x, y);
			}

			auto v2 = r2.getPoint(focusLen);

			auto vec = v2 - v1;
			
			imp_.UpdateData(target - vec, pos - vec);

			imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();
		}
	}
}

void MayaCamera::ReAttach()
{
	auto& imp_ = *ImpUPtr_;

	imp_.PositionNode_->attachObject(imp_.Camera_);
}