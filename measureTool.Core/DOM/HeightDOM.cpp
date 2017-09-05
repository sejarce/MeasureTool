#include "HeightDOM.h"

#include "Ogre.h"

class	HeightDOM::Imp
{
public:
	
	Ogre::Vector3	P1_;
	Ogre::Vector3	P2_;
};

HeightDOM::HeightDOM(const DocumentSPtr& doc, uint32_t index) :IDOM(doc, index)
, ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;
}

HeightDOM::~HeightDOM()
{

}

IDOM::EDOMType HeightDOM::GetType() const
{
	auto& imp_ = *ImpUPtr_;

	return IDOM::EDT_Height;
}

void HeightDOM::SetPoints(const Ogre::Vector3& p1, const Ogre::Vector3& p2)
{
	auto& imp_ = *ImpUPtr_;

	imp_.P1_ = p1;
	imp_.P2_ = p2;

	UpdateDOM();
}

const Ogre::Vector3& HeightDOM::GetPoint1() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.P1_;
}

const Ogre::Vector3& HeightDOM::GetPoint2() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.P2_;
}

void HeightDOM::OnUpdate(float& val, PntList& pntList)
{
	auto& imp_ = *ImpUPtr_;

	val = abs(imp_.P1_.y - imp_.P2_.y);

	pntList.clear();
	pntList.push_back(imp_.P1_);
	pntList.push_back(imp_.P2_);
}

void HeightDOM::OnDeserialize(const std::string& buf)
{
	auto& imp_ = *ImpUPtr_;

	imp_.P1_ = GetPntList()[0];
	imp_.P2_ = GetPntList()[1];
}

void HeightDOM::OnDeserialize(const PntList& pList)
{
	auto& imp_ = *ImpUPtr_;

	imp_.P1_ = pList.front();
	imp_.P2_ = pList.back();
}
