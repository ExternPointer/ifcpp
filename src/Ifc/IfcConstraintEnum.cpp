/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcConstraintEnum.h"

// TYPE IfcConstraintEnum = ENUMERATION OF	(ADVISORY	,HARD	,SOFT	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcConstraintEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCONSTRAINTENUM("; }
	switch( m_enum )
	{
		case ENUM_ADVISORY:	stream << ".ADVISORY."; break;
		case ENUM_HARD:	stream << ".HARD."; break;
		case ENUM_SOFT:	stream << ".SOFT."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcConstraintEnum> IFC4X3::IfcConstraintEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcConstraintEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcConstraintEnum>(); }
	shared_ptr<IfcConstraintEnum> type_object( new IfcConstraintEnum() );
	if( std_iequal( arg, ".ADVISORY." ) )
	{
		type_object->m_enum = IfcConstraintEnum::ENUM_ADVISORY;
	}
	else if( std_iequal( arg, ".HARD." ) )
	{
		type_object->m_enum = IfcConstraintEnum::ENUM_HARD;
	}
	else if( std_iequal( arg, ".SOFT." ) )
	{
		type_object->m_enum = IfcConstraintEnum::ENUM_SOFT;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcConstraintEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcConstraintEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
