/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcHumidifierTypeEnum.h"

// TYPE IfcHumidifierTypeEnum = ENUMERATION OF	(ADIABATICAIRWASHER	,ADIABATICATOMIZING	,ADIABATICCOMPRESSEDAIRNOZZLE	,ADIABATICPAN	,ADIABATICRIGIDMEDIA	,ADIABATICULTRASONIC	,ADIABATICWETTEDELEMENT	,ASSISTEDBUTANE	,ASSISTEDELECTRIC	,ASSISTEDNATURALGAS	,ASSISTEDPROPANE	,ASSISTEDSTEAM	,STEAMINJECTION	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcHumidifierTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCHUMIDIFIERTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_ADIABATICAIRWASHER:	stream << ".ADIABATICAIRWASHER."; break;
		case ENUM_ADIABATICATOMIZING:	stream << ".ADIABATICATOMIZING."; break;
		case ENUM_ADIABATICCOMPRESSEDAIRNOZZLE:	stream << ".ADIABATICCOMPRESSEDAIRNOZZLE."; break;
		case ENUM_ADIABATICPAN:	stream << ".ADIABATICPAN."; break;
		case ENUM_ADIABATICRIGIDMEDIA:	stream << ".ADIABATICRIGIDMEDIA."; break;
		case ENUM_ADIABATICULTRASONIC:	stream << ".ADIABATICULTRASONIC."; break;
		case ENUM_ADIABATICWETTEDELEMENT:	stream << ".ADIABATICWETTEDELEMENT."; break;
		case ENUM_ASSISTEDBUTANE:	stream << ".ASSISTEDBUTANE."; break;
		case ENUM_ASSISTEDELECTRIC:	stream << ".ASSISTEDELECTRIC."; break;
		case ENUM_ASSISTEDNATURALGAS:	stream << ".ASSISTEDNATURALGAS."; break;
		case ENUM_ASSISTEDPROPANE:	stream << ".ASSISTEDPROPANE."; break;
		case ENUM_ASSISTEDSTEAM:	stream << ".ASSISTEDSTEAM."; break;
		case ENUM_STEAMINJECTION:	stream << ".STEAMINJECTION."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcHumidifierTypeEnum> IFC4X3::IfcHumidifierTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcHumidifierTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcHumidifierTypeEnum>(); }
	shared_ptr<IfcHumidifierTypeEnum> type_object( new IfcHumidifierTypeEnum() );
	if( std_iequal( arg, ".ADIABATICAIRWASHER." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICAIRWASHER;
	}
	else if( std_iequal( arg, ".ADIABATICATOMIZING." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICATOMIZING;
	}
	else if( std_iequal( arg, ".ADIABATICCOMPRESSEDAIRNOZZLE." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICCOMPRESSEDAIRNOZZLE;
	}
	else if( std_iequal( arg, ".ADIABATICPAN." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICPAN;
	}
	else if( std_iequal( arg, ".ADIABATICRIGIDMEDIA." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICRIGIDMEDIA;
	}
	else if( std_iequal( arg, ".ADIABATICULTRASONIC." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICULTRASONIC;
	}
	else if( std_iequal( arg, ".ADIABATICWETTEDELEMENT." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ADIABATICWETTEDELEMENT;
	}
	else if( std_iequal( arg, ".ASSISTEDBUTANE." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ASSISTEDBUTANE;
	}
	else if( std_iequal( arg, ".ASSISTEDELECTRIC." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ASSISTEDELECTRIC;
	}
	else if( std_iequal( arg, ".ASSISTEDNATURALGAS." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ASSISTEDNATURALGAS;
	}
	else if( std_iequal( arg, ".ASSISTEDPROPANE." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ASSISTEDPROPANE;
	}
	else if( std_iequal( arg, ".ASSISTEDSTEAM." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_ASSISTEDSTEAM;
	}
	else if( std_iequal( arg, ".STEAMINJECTION." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_STEAMINJECTION;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcHumidifierTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
