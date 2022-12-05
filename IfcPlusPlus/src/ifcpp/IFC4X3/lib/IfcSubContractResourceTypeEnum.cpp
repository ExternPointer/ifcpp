/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcSubContractResourceTypeEnum.h"

// TYPE IfcSubContractResourceTypeEnum = ENUMERATION OF	(PURCHASE	,WORK	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcSubContractResourceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCSUBCONTRACTRESOURCETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_PURCHASE:	stream << ".PURCHASE."; break;
		case ENUM_WORK:	stream << ".WORK."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcSubContractResourceTypeEnum> IFC4X3::IfcSubContractResourceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcSubContractResourceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcSubContractResourceTypeEnum>(); }
	shared_ptr<IfcSubContractResourceTypeEnum> type_object( new IfcSubContractResourceTypeEnum() );
	if( std_iequal( arg, ".PURCHASE." ) )
	{
		type_object->m_enum = IfcSubContractResourceTypeEnum::ENUM_PURCHASE;
	}
	else if( std_iequal( arg, ".WORK." ) )
	{
		type_object->m_enum = IfcSubContractResourceTypeEnum::ENUM_WORK;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcSubContractResourceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcSubContractResourceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
