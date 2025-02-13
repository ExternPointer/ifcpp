/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcCommunicationsApplianceTypeEnum.h"

// TYPE IfcCommunicationsApplianceTypeEnum = ENUMERATION OF	(ANTENNA	,AUTOMATON	,COMPUTER	,FAX	,GATEWAY	,INTELLIGENTPERIPHERAL	,IPNETWORKEQUIPMENT	,LINESIDEELECTRONICUNIT	,MODEM	,NETWORKAPPLIANCE	,NETWORKBRIDGE	,NETWORKHUB	,OPTICALLINETERMINAL	,OPTICALNETWORKUNIT	,PRINTER	,RADIOBLOCKCENTER	,REPEATER	,ROUTER	,SCANNER	,TELECOMMAND	,TELEPHONYEXCHANGE	,TRANSITIONCOMPONENT	,TRANSPONDER	,TRANSPORTEQUIPMENT	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcCommunicationsApplianceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCOMMUNICATIONSAPPLIANCETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_ANTENNA:	stream << ".ANTENNA."; break;
		case ENUM_AUTOMATON:	stream << ".AUTOMATON."; break;
		case ENUM_COMPUTER:	stream << ".COMPUTER."; break;
		case ENUM_FAX:	stream << ".FAX."; break;
		case ENUM_GATEWAY:	stream << ".GATEWAY."; break;
		case ENUM_INTELLIGENTPERIPHERAL:	stream << ".INTELLIGENTPERIPHERAL."; break;
		case ENUM_IPNETWORKEQUIPMENT:	stream << ".IPNETWORKEQUIPMENT."; break;
		case ENUM_LINESIDEELECTRONICUNIT:	stream << ".LINESIDEELECTRONICUNIT."; break;
		case ENUM_MODEM:	stream << ".MODEM."; break;
		case ENUM_NETWORKAPPLIANCE:	stream << ".NETWORKAPPLIANCE."; break;
		case ENUM_NETWORKBRIDGE:	stream << ".NETWORKBRIDGE."; break;
		case ENUM_NETWORKHUB:	stream << ".NETWORKHUB."; break;
		case ENUM_OPTICALLINETERMINAL:	stream << ".OPTICALLINETERMINAL."; break;
		case ENUM_OPTICALNETWORKUNIT:	stream << ".OPTICALNETWORKUNIT."; break;
		case ENUM_PRINTER:	stream << ".PRINTER."; break;
		case ENUM_RADIOBLOCKCENTER:	stream << ".RADIOBLOCKCENTER."; break;
		case ENUM_REPEATER:	stream << ".REPEATER."; break;
		case ENUM_ROUTER:	stream << ".ROUTER."; break;
		case ENUM_SCANNER:	stream << ".SCANNER."; break;
		case ENUM_TELECOMMAND:	stream << ".TELECOMMAND."; break;
		case ENUM_TELEPHONYEXCHANGE:	stream << ".TELEPHONYEXCHANGE."; break;
		case ENUM_TRANSITIONCOMPONENT:	stream << ".TRANSITIONCOMPONENT."; break;
		case ENUM_TRANSPONDER:	stream << ".TRANSPONDER."; break;
		case ENUM_TRANSPORTEQUIPMENT:	stream << ".TRANSPORTEQUIPMENT."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcCommunicationsApplianceTypeEnum> IFC4X3::IfcCommunicationsApplianceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcCommunicationsApplianceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcCommunicationsApplianceTypeEnum>(); }
	shared_ptr<IfcCommunicationsApplianceTypeEnum> type_object( new IfcCommunicationsApplianceTypeEnum() );
	if( std_iequal( arg, ".ANTENNA." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_ANTENNA;
	}
	else if( std_iequal( arg, ".AUTOMATON." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_AUTOMATON;
	}
	else if( std_iequal( arg, ".COMPUTER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_COMPUTER;
	}
	else if( std_iequal( arg, ".FAX." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_FAX;
	}
	else if( std_iequal( arg, ".GATEWAY." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_GATEWAY;
	}
	else if( std_iequal( arg, ".INTELLIGENTPERIPHERAL." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_INTELLIGENTPERIPHERAL;
	}
	else if( std_iequal( arg, ".IPNETWORKEQUIPMENT." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_IPNETWORKEQUIPMENT;
	}
	else if( std_iequal( arg, ".LINESIDEELECTRONICUNIT." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_LINESIDEELECTRONICUNIT;
	}
	else if( std_iequal( arg, ".MODEM." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_MODEM;
	}
	else if( std_iequal( arg, ".NETWORKAPPLIANCE." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_NETWORKAPPLIANCE;
	}
	else if( std_iequal( arg, ".NETWORKBRIDGE." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_NETWORKBRIDGE;
	}
	else if( std_iequal( arg, ".NETWORKHUB." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_NETWORKHUB;
	}
	else if( std_iequal( arg, ".OPTICALLINETERMINAL." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_OPTICALLINETERMINAL;
	}
	else if( std_iequal( arg, ".OPTICALNETWORKUNIT." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_OPTICALNETWORKUNIT;
	}
	else if( std_iequal( arg, ".PRINTER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_PRINTER;
	}
	else if( std_iequal( arg, ".RADIOBLOCKCENTER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_RADIOBLOCKCENTER;
	}
	else if( std_iequal( arg, ".REPEATER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_REPEATER;
	}
	else if( std_iequal( arg, ".ROUTER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_ROUTER;
	}
	else if( std_iequal( arg, ".SCANNER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_SCANNER;
	}
	else if( std_iequal( arg, ".TELECOMMAND." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_TELECOMMAND;
	}
	else if( std_iequal( arg, ".TELEPHONYEXCHANGE." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_TELEPHONYEXCHANGE;
	}
	else if( std_iequal( arg, ".TRANSITIONCOMPONENT." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_TRANSITIONCOMPONENT;
	}
	else if( std_iequal( arg, ".TRANSPONDER." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_TRANSPONDER;
	}
	else if( std_iequal( arg, ".TRANSPORTEQUIPMENT." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_TRANSPORTEQUIPMENT;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcCommunicationsApplianceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
