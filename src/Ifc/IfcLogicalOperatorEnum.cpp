/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcLogicalOperatorEnum.h"

// TYPE IfcLogicalOperatorEnum = ENUMERATION OF	(LOGICALAND	,LOGICALNOTAND	,LOGICALNOTOR	,LOGICALOR	,LOGICALXOR);
void IFC4X3::IfcLogicalOperatorEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLOGICALOPERATORENUM("; }
	switch( m_enum )
	{
		case ENUM_LOGICALAND:	stream << ".LOGICALAND."; break;
		case ENUM_LOGICALNOTAND:	stream << ".LOGICALNOTAND."; break;
		case ENUM_LOGICALNOTOR:	stream << ".LOGICALNOTOR."; break;
		case ENUM_LOGICALOR:	stream << ".LOGICALOR."; break;
		case ENUM_LOGICALXOR:	stream << ".LOGICALXOR."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcLogicalOperatorEnum> IFC4X3::IfcLogicalOperatorEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLogicalOperatorEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcLogicalOperatorEnum>(); }
	shared_ptr<IfcLogicalOperatorEnum> type_object( new IfcLogicalOperatorEnum() );
	if( std_iequal( arg, ".LOGICALAND." ) )
	{
		type_object->m_enum = IfcLogicalOperatorEnum::ENUM_LOGICALAND;
	}
	else if( std_iequal( arg, ".LOGICALNOTAND." ) )
	{
		type_object->m_enum = IfcLogicalOperatorEnum::ENUM_LOGICALNOTAND;
	}
	else if( std_iequal( arg, ".LOGICALNOTOR." ) )
	{
		type_object->m_enum = IfcLogicalOperatorEnum::ENUM_LOGICALNOTOR;
	}
	else if( std_iequal( arg, ".LOGICALOR." ) )
	{
		type_object->m_enum = IfcLogicalOperatorEnum::ENUM_LOGICALOR;
	}
	else if( std_iequal( arg, ".LOGICALXOR." ) )
	{
		type_object->m_enum = IfcLogicalOperatorEnum::ENUM_LOGICALXOR;
	}
	return type_object;
}
