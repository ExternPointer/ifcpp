/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcGridTypeEnum.h"

// TYPE IfcGridTypeEnum = ENUMERATION OF	(IRREGULAR	,RADIAL	,RECTANGULAR	,TRIANGULAR	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcGridTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCGRIDTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_IRREGULAR:	stream << ".IRREGULAR."; break;
		case ENUM_RADIAL:	stream << ".RADIAL."; break;
		case ENUM_RECTANGULAR:	stream << ".RECTANGULAR."; break;
		case ENUM_TRIANGULAR:	stream << ".TRIANGULAR."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcGridTypeEnum> IFC4X3::IfcGridTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcGridTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcGridTypeEnum>(); }
	shared_ptr<IfcGridTypeEnum> type_object( new IfcGridTypeEnum() );
	if( std_iequal( arg, ".IRREGULAR." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_IRREGULAR;
	}
	else if( std_iequal( arg, ".RADIAL." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_RADIAL;
	}
	else if( std_iequal( arg, ".RECTANGULAR." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_RECTANGULAR;
	}
	else if( std_iequal( arg, ".TRIANGULAR." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_TRIANGULAR;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcGridTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
