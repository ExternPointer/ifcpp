/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMarinePartTypeEnum.h"

// TYPE IfcMarinePartTypeEnum = ENUMERATION OF	(ABOVEWATERLINE	,ANCHORAGE	,APPROACHCHANNEL	,BELOWWATERLINE	,BERTHINGSTRUCTURE	,CHAMBER	,CILL_LEVEL	,COPELEVEL	,CORE	,CREST	,GATEHEAD	,GUDINGSTRUCTURE	,HIGHWATERLINE	,LANDFIELD	,LEEWARDSIDE	,LOWWATERLINE	,MANUFACTURING	,NAVIGATIONALAREA	,PROTECTION	,SHIPTRANSFER	,STORAGEAREA	,VEHICLESERVICING	,WATERFIELD	,WEATHERSIDE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcMarinePartTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCMARINEPARTTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_ABOVEWATERLINE:	stream << ".ABOVEWATERLINE."; break;
		case ENUM_ANCHORAGE:	stream << ".ANCHORAGE."; break;
		case ENUM_APPROACHCHANNEL:	stream << ".APPROACHCHANNEL."; break;
		case ENUM_BELOWWATERLINE:	stream << ".BELOWWATERLINE."; break;
		case ENUM_BERTHINGSTRUCTURE:	stream << ".BERTHINGSTRUCTURE."; break;
		case ENUM_CHAMBER:	stream << ".CHAMBER."; break;
		case ENUM_CILL_LEVEL:	stream << ".CILL_LEVEL."; break;
		case ENUM_COPELEVEL:	stream << ".COPELEVEL."; break;
		case ENUM_CORE:	stream << ".CORE."; break;
		case ENUM_CREST:	stream << ".CREST."; break;
		case ENUM_GATEHEAD:	stream << ".GATEHEAD."; break;
		case ENUM_GUDINGSTRUCTURE:	stream << ".GUDINGSTRUCTURE."; break;
		case ENUM_HIGHWATERLINE:	stream << ".HIGHWATERLINE."; break;
		case ENUM_LANDFIELD:	stream << ".LANDFIELD."; break;
		case ENUM_LEEWARDSIDE:	stream << ".LEEWARDSIDE."; break;
		case ENUM_LOWWATERLINE:	stream << ".LOWWATERLINE."; break;
		case ENUM_MANUFACTURING:	stream << ".MANUFACTURING."; break;
		case ENUM_NAVIGATIONALAREA:	stream << ".NAVIGATIONALAREA."; break;
		case ENUM_PROTECTION:	stream << ".PROTECTION."; break;
		case ENUM_SHIPTRANSFER:	stream << ".SHIPTRANSFER."; break;
		case ENUM_STORAGEAREA:	stream << ".STORAGEAREA."; break;
		case ENUM_VEHICLESERVICING:	stream << ".VEHICLESERVICING."; break;
		case ENUM_WATERFIELD:	stream << ".WATERFIELD."; break;
		case ENUM_WEATHERSIDE:	stream << ".WEATHERSIDE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcMarinePartTypeEnum> IFC4X3::IfcMarinePartTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcMarinePartTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcMarinePartTypeEnum>(); }
	shared_ptr<IfcMarinePartTypeEnum> type_object( new IfcMarinePartTypeEnum() );
	if( std_iequal( arg, ".ABOVEWATERLINE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_ABOVEWATERLINE;
	}
	else if( std_iequal( arg, ".ANCHORAGE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_ANCHORAGE;
	}
	else if( std_iequal( arg, ".APPROACHCHANNEL." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_APPROACHCHANNEL;
	}
	else if( std_iequal( arg, ".BELOWWATERLINE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_BELOWWATERLINE;
	}
	else if( std_iequal( arg, ".BERTHINGSTRUCTURE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_BERTHINGSTRUCTURE;
	}
	else if( std_iequal( arg, ".CHAMBER." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_CHAMBER;
	}
	else if( std_iequal( arg, ".CILL_LEVEL." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_CILL_LEVEL;
	}
	else if( std_iequal( arg, ".COPELEVEL." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_COPELEVEL;
	}
	else if( std_iequal( arg, ".CORE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_CORE;
	}
	else if( std_iequal( arg, ".CREST." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_CREST;
	}
	else if( std_iequal( arg, ".GATEHEAD." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_GATEHEAD;
	}
	else if( std_iequal( arg, ".GUDINGSTRUCTURE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_GUDINGSTRUCTURE;
	}
	else if( std_iequal( arg, ".HIGHWATERLINE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_HIGHWATERLINE;
	}
	else if( std_iequal( arg, ".LANDFIELD." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_LANDFIELD;
	}
	else if( std_iequal( arg, ".LEEWARDSIDE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_LEEWARDSIDE;
	}
	else if( std_iequal( arg, ".LOWWATERLINE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_LOWWATERLINE;
	}
	else if( std_iequal( arg, ".MANUFACTURING." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_MANUFACTURING;
	}
	else if( std_iequal( arg, ".NAVIGATIONALAREA." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_NAVIGATIONALAREA;
	}
	else if( std_iequal( arg, ".PROTECTION." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_PROTECTION;
	}
	else if( std_iequal( arg, ".SHIPTRANSFER." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_SHIPTRANSFER;
	}
	else if( std_iequal( arg, ".STORAGEAREA." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_STORAGEAREA;
	}
	else if( std_iequal( arg, ".VEHICLESERVICING." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_VEHICLESERVICING;
	}
	else if( std_iequal( arg, ".WATERFIELD." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_WATERFIELD;
	}
	else if( std_iequal( arg, ".WEATHERSIDE." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_WEATHERSIDE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcMarinePartTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
