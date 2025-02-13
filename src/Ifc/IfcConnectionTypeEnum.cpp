/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcConnectionTypeEnum.h"

// TYPE IfcConnectionTypeEnum = ENUMERATION OF	(ATEND	,ATPATH	,ATSTART	,NOTDEFINED);
void IFC4X3::IfcConnectionTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCONNECTIONTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_ATEND:	stream << ".ATEND."; break;
		case ENUM_ATPATH:	stream << ".ATPATH."; break;
		case ENUM_ATSTART:	stream << ".ATSTART."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcConnectionTypeEnum> IFC4X3::IfcConnectionTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcConnectionTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcConnectionTypeEnum>(); }
	shared_ptr<IfcConnectionTypeEnum> type_object( new IfcConnectionTypeEnum() );
	if( std_iequal( arg, ".ATEND." ) )
	{
		type_object->m_enum = IfcConnectionTypeEnum::ENUM_ATEND;
	}
	else if( std_iequal( arg, ".ATPATH." ) )
	{
		type_object->m_enum = IfcConnectionTypeEnum::ENUM_ATPATH;
	}
	else if( std_iequal( arg, ".ATSTART." ) )
	{
		type_object->m_enum = IfcConnectionTypeEnum::ENUM_ATSTART;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcConnectionTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
