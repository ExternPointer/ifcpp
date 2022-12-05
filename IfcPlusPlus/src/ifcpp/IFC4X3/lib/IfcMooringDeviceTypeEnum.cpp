/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcMooringDeviceTypeEnum.h"

// TYPE IfcMooringDeviceTypeEnum = ENUMERATION OF	(BOLLARD	,LINETENSIONER	,MAGNETICDEVICE	,MOORINGHOOKS	,VACUUMDEVICE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcMooringDeviceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCMOORINGDEVICETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_BOLLARD:	stream << ".BOLLARD."; break;
		case ENUM_LINETENSIONER:	stream << ".LINETENSIONER."; break;
		case ENUM_MAGNETICDEVICE:	stream << ".MAGNETICDEVICE."; break;
		case ENUM_MOORINGHOOKS:	stream << ".MOORINGHOOKS."; break;
		case ENUM_VACUUMDEVICE:	stream << ".VACUUMDEVICE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcMooringDeviceTypeEnum> IFC4X3::IfcMooringDeviceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcMooringDeviceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcMooringDeviceTypeEnum>(); }
	shared_ptr<IfcMooringDeviceTypeEnum> type_object( new IfcMooringDeviceTypeEnum() );
	if( std_iequal( arg, ".BOLLARD." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_BOLLARD;
	}
	else if( std_iequal( arg, ".LINETENSIONER." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_LINETENSIONER;
	}
	else if( std_iequal( arg, ".MAGNETICDEVICE." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_MAGNETICDEVICE;
	}
	else if( std_iequal( arg, ".MOORINGHOOKS." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_MOORINGHOOKS;
	}
	else if( std_iequal( arg, ".VACUUMDEVICE." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_VACUUMDEVICE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcMooringDeviceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
