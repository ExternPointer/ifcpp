/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcBeamTypeEnum.h"

// TYPE IfcBeamTypeEnum = ENUMERATION OF	(BEAM	,CORNICE	,DIAPHRAGM	,EDGEBEAM	,GIRDER_SEGMENT	,HATSTONE	,HOLLOWCORE	,JOIST	,LINTEL	,PIERCAP	,SPANDREL	,T_BEAM	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcBeamTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCBEAMTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_BEAM:	stream << ".BEAM."; break;
		case ENUM_CORNICE:	stream << ".CORNICE."; break;
		case ENUM_DIAPHRAGM:	stream << ".DIAPHRAGM."; break;
		case ENUM_EDGEBEAM:	stream << ".EDGEBEAM."; break;
		case ENUM_GIRDER_SEGMENT:	stream << ".GIRDER_SEGMENT."; break;
		case ENUM_HATSTONE:	stream << ".HATSTONE."; break;
		case ENUM_HOLLOWCORE:	stream << ".HOLLOWCORE."; break;
		case ENUM_JOIST:	stream << ".JOIST."; break;
		case ENUM_LINTEL:	stream << ".LINTEL."; break;
		case ENUM_PIERCAP:	stream << ".PIERCAP."; break;
		case ENUM_SPANDREL:	stream << ".SPANDREL."; break;
		case ENUM_T_BEAM:	stream << ".T_BEAM."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcBeamTypeEnum> IFC4X3::IfcBeamTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcBeamTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcBeamTypeEnum>(); }
	shared_ptr<IfcBeamTypeEnum> type_object( new IfcBeamTypeEnum() );
	if( std_iequal( arg, ".BEAM." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_BEAM;
	}
	else if( std_iequal( arg, ".CORNICE." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_CORNICE;
	}
	else if( std_iequal( arg, ".DIAPHRAGM." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_DIAPHRAGM;
	}
	else if( std_iequal( arg, ".EDGEBEAM." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_EDGEBEAM;
	}
	else if( std_iequal( arg, ".GIRDER_SEGMENT." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_GIRDER_SEGMENT;
	}
	else if( std_iequal( arg, ".HATSTONE." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_HATSTONE;
	}
	else if( std_iequal( arg, ".HOLLOWCORE." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_HOLLOWCORE;
	}
	else if( std_iequal( arg, ".JOIST." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_JOIST;
	}
	else if( std_iequal( arg, ".LINTEL." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_LINTEL;
	}
	else if( std_iequal( arg, ".PIERCAP." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_PIERCAP;
	}
	else if( std_iequal( arg, ".SPANDREL." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_SPANDREL;
	}
	else if( std_iequal( arg, ".T_BEAM." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_T_BEAM;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcBeamTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
