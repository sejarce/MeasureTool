#include "DataWrapper.h"

#include "DOM/Document.h"
#include "DOM/IDOM.h"


DataWrapper::DataWrapper(IDOMSPtr dom, QString image_n, QString image_s, bool neworold, bool saved /*= false*/) : m_DOM(dom)
, m_image_n(image_n)
, m_image_s(image_s)
, m_NeworOld(neworold)
{

}

DataWrapper::DataWrapper(QObject* parent /*= nullptr*/) : QObject(parent)
{

}

DataWrapper::~DataWrapper()
{
	
}


QString DataWrapper::name()
{
	if ( m_DOM.lock()->GetName().empty() )
	{
		return type();
	}
	else
	{
		return QString::fromStdWString(m_DOM.lock()->GetName());
	}
}


void DataWrapper::setName(QString name)
{
	m_DOM.lock()->SetName(name.toStdWString(), false);
}


float DataWrapper::value()
{
	return m_DOM.lock()->GetValue();
}


QString DataWrapper::type()
{
	switch ( m_DOM.lock()->GetType() )
	{
	case IDOM::EDT_Height:
	return QString::fromLocal8Bit("高度");
	case IDOM::EDT_LineLength:
	return QString::fromLocal8Bit("长度");
	case IDOM::EDT_CurveLength:
	return QString::fromLocal8Bit("表面长度");
	case IDOM::EDT_Dimension:
	return QString::fromLocal8Bit("表面围度");
	case IDOM::EDT_ConvexDimension:
	return QString::fromLocal8Bit("围度");
	default:
	break;
	}
	return QString("未知类型");
}


QString DataWrapper::editTime()
{
	std::time_t tim = static_cast<std::time_t>( m_DOM.lock()->GetLastEditTime() );
	//time(&tim);
	struct tm currtime;
	currtime = *localtime(&tim);
	char timestr[64];
	strftime(timestr, 64, "%Y/%m/%d %H:%M", &currtime);
	return QString(timestr);
}


QString DataWrapper::image_n()
{
	return m_image_n;
}

QString DataWrapper::image_s()
{
	return m_image_s;
}

bool DataWrapper::isCreate() const
{
	auto dom = m_DOM.lock();
	return dom->TestState(IDOM::EDS_New) && !dom->TestState(IDOM::EDS_Editable);
}

bool DataWrapper::isDirty() const
{
	auto dom = m_DOM.lock();
	return dom->isDirty();
}

bool DataWrapper::neworOld()
{
	return m_NeworOld;
}

void DataWrapper::setNeworOld(bool flag)
{
	m_NeworOld = flag;
}

bool DataWrapper::saved()
{
	auto dom = m_DOM.lock();
	return dom->isSaved();
}

void DataWrapper::setSaved(bool flag)
{
	auto dom = m_DOM.lock();
	return dom->SetSaved(flag);
}

IDOMSPtr DataWrapper::DOM()
{
	return m_DOM.lock();
}

void DataWrapper::setDOM(IDOMSPtr dom)
{
	m_DOM = dom;
}

#include "moc_DataWrapper.cpp"
