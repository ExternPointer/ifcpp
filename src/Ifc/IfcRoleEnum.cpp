/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcRoleEnum.h"

// TYPE IfcRoleEnum = ENUMERATION OF	(ARCHITECT	,BUILDINGOPERATOR	,BUILDINGOWNER	,CIVILENGINEER	,CLIENT	,COMMISSIONINGENGINEER	,CONSTRUCTIONMANAGER	,CONSULTANT	,CONTRACTOR	,COSTENGINEER	,ELECTRICALENGINEER	,ENGINEER	,FACILITIESMANAGER	,FIELDCONSTRUCTIONMANAGER	,MANUFACTURER	,MECHANICALENGINEER	,OWNER	,PROJECTMANAGER	,RESELLER	,STRUCTURALENGINEER	,SUBCONTRACTOR	,SUPPLIER	,USERDEFINED);
void IFC4X3::IfcRoleEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCROLEENUM("; }
	switch( m_enum )
	{
		case ENUM_ARCHITECT:	stream << ".ARCHITECT."; break;
		case ENUM_BUILDINGOPERATOR:	stream << ".BUILDINGOPERATOR."; break;
		case ENUM_BUILDINGOWNER:	stream << ".BUILDINGOWNER."; break;
		case ENUM_CIVILENGINEER:	stream << ".CIVILENGINEER."; break;
		case ENUM_CLIENT:	stream << ".CLIENT."; break;
		case ENUM_COMMISSIONINGENGINEER:	stream << ".COMMISSIONINGENGINEER."; break;
		case ENUM_CONSTRUCTIONMANAGER:	stream << ".CONSTRUCTIONMANAGER."; break;
		case ENUM_CONSULTANT:	stream << ".CONSULTANT."; break;
		case ENUM_CONTRACTOR:	stream << ".CONTRACTOR."; break;
		case ENUM_COSTENGINEER:	stream << ".COSTENGINEER."; break;
		case ENUM_ELECTRICALENGINEER:	stream << ".ELECTRICALENGINEER."; break;
		case ENUM_ENGINEER:	stream << ".ENGINEER."; break;
		case ENUM_FACILITIESMANAGER:	stream << ".FACILITIESMANAGER."; break;
		case ENUM_FIELDCONSTRUCTIONMANAGER:	stream << ".FIELDCONSTRUCTIONMANAGER."; break;
		case ENUM_MANUFACTURER:	stream << ".MANUFACTURER."; break;
		case ENUM_MECHANICALENGINEER:	stream << ".MECHANICALENGINEER."; break;
		case ENUM_OWNER:	stream << ".OWNER."; break;
		case ENUM_PROJECTMANAGER:	stream << ".PROJECTMANAGER."; break;
		case ENUM_RESELLER:	stream << ".RESELLER."; break;
		case ENUM_STRUCTURALENGINEER:	stream << ".STRUCTURALENGINEER."; break;
		case ENUM_SUBCONTRACTOR:	stream << ".SUBCONTRACTOR."; break;
		case ENUM_SUPPLIER:	stream << ".SUPPLIER."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcRoleEnum> IFC4X3::IfcRoleEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcRoleEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcRoleEnum>(); }
	shared_ptr<IfcRoleEnum> type_object( new IfcRoleEnum() );
	if( std_iequal( arg, ".ARCHITECT." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_ARCHITECT;
	}
	else if( std_iequal( arg, ".BUILDINGOPERATOR." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_BUILDINGOPERATOR;
	}
	else if( std_iequal( arg, ".BUILDINGOWNER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_BUILDINGOWNER;
	}
	else if( std_iequal( arg, ".CIVILENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_CIVILENGINEER;
	}
	else if( std_iequal( arg, ".CLIENT." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_CLIENT;
	}
	else if( std_iequal( arg, ".COMMISSIONINGENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_COMMISSIONINGENGINEER;
	}
	else if( std_iequal( arg, ".CONSTRUCTIONMANAGER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_CONSTRUCTIONMANAGER;
	}
	else if( std_iequal( arg, ".CONSULTANT." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_CONSULTANT;
	}
	else if( std_iequal( arg, ".CONTRACTOR." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_CONTRACTOR;
	}
	else if( std_iequal( arg, ".COSTENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_COSTENGINEER;
	}
	else if( std_iequal( arg, ".ELECTRICALENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_ELECTRICALENGINEER;
	}
	else if( std_iequal( arg, ".ENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_ENGINEER;
	}
	else if( std_iequal( arg, ".FACILITIESMANAGER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_FACILITIESMANAGER;
	}
	else if( std_iequal( arg, ".FIELDCONSTRUCTIONMANAGER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_FIELDCONSTRUCTIONMANAGER;
	}
	else if( std_iequal( arg, ".MANUFACTURER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_MANUFACTURER;
	}
	else if( std_iequal( arg, ".MECHANICALENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_MECHANICALENGINEER;
	}
	else if( std_iequal( arg, ".OWNER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_OWNER;
	}
	else if( std_iequal( arg, ".PROJECTMANAGER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_PROJECTMANAGER;
	}
	else if( std_iequal( arg, ".RESELLER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_RESELLER;
	}
	else if( std_iequal( arg, ".STRUCTURALENGINEER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_STRUCTURALENGINEER;
	}
	else if( std_iequal( arg, ".SUBCONTRACTOR." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_SUBCONTRACTOR;
	}
	else if( std_iequal( arg, ".SUPPLIER." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_SUPPLIER;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcRoleEnum::ENUM_USERDEFINED;
	}
	return type_object;
}
