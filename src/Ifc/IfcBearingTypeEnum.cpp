/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcBearingTypeEnum.h"

// TYPE IfcBearingTypeEnum = ENUMERATION OF	(CYLINDRICAL	,DISK	,ELASTOMERIC	,GUIDE	,POT	,ROCKER	,ROLLER	,SPHERICAL	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcBearingTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCBEARINGTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_CYLINDRICAL:	stream << ".CYLINDRICAL."; break;
		case ENUM_DISK:	stream << ".DISK."; break;
		case ENUM_ELASTOMERIC:	stream << ".ELASTOMERIC."; break;
		case ENUM_GUIDE:	stream << ".GUIDE."; break;
		case ENUM_POT:	stream << ".POT."; break;
		case ENUM_ROCKER:	stream << ".ROCKER."; break;
		case ENUM_ROLLER:	stream << ".ROLLER."; break;
		case ENUM_SPHERICAL:	stream << ".SPHERICAL."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcBearingTypeEnum> IFC4X3::IfcBearingTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcBearingTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcBearingTypeEnum>(); }
	shared_ptr<IfcBearingTypeEnum> type_object( new IfcBearingTypeEnum() );
	if( std_iequal( arg, ".CYLINDRICAL." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_CYLINDRICAL;
	}
	else if( std_iequal( arg, ".DISK." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_DISK;
	}
	else if( std_iequal( arg, ".ELASTOMERIC." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_ELASTOMERIC;
	}
	else if( std_iequal( arg, ".GUIDE." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_GUIDE;
	}
	else if( std_iequal( arg, ".POT." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_POT;
	}
	else if( std_iequal( arg, ".ROCKER." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_ROCKER;
	}
	else if( std_iequal( arg, ".ROLLER." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_ROLLER;
	}
	else if( std_iequal( arg, ".SPHERICAL." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_SPHERICAL;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcBearingTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
