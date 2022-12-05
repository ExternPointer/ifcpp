/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcSIUnitName.h"

// TYPE IfcSIUnitName = ENUMERATION OF	(AMPERE	,BECQUEREL	,CANDELA	,COULOMB	,CUBIC_METRE	,DEGREE_CELSIUS	,FARAD	,GRAM	,GRAY	,HENRY	,HERTZ	,JOULE	,KELVIN	,LUMEN	,LUX	,METRE	,MOLE	,NEWTON	,OHM	,PASCAL	,RADIAN	,SECOND	,SIEMENS	,SIEVERT	,SQUARE_METRE	,STERADIAN	,TESLA	,VOLT	,WATT	,WEBER);
void IFC4X3::IfcSIUnitName::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCSIUNITNAME("; }
	switch( m_enum )
	{
		case ENUM_AMPERE:	stream << ".AMPERE."; break;
		case ENUM_BECQUEREL:	stream << ".BECQUEREL."; break;
		case ENUM_CANDELA:	stream << ".CANDELA."; break;
		case ENUM_COULOMB:	stream << ".COULOMB."; break;
		case ENUM_CUBIC_METRE:	stream << ".CUBIC_METRE."; break;
		case ENUM_DEGREE_CELSIUS:	stream << ".DEGREE_CELSIUS."; break;
		case ENUM_FARAD:	stream << ".FARAD."; break;
		case ENUM_GRAM:	stream << ".GRAM."; break;
		case ENUM_GRAY:	stream << ".GRAY."; break;
		case ENUM_HENRY:	stream << ".HENRY."; break;
		case ENUM_HERTZ:	stream << ".HERTZ."; break;
		case ENUM_JOULE:	stream << ".JOULE."; break;
		case ENUM_KELVIN:	stream << ".KELVIN."; break;
		case ENUM_LUMEN:	stream << ".LUMEN."; break;
		case ENUM_LUX:	stream << ".LUX."; break;
		case ENUM_METRE:	stream << ".METRE."; break;
		case ENUM_MOLE:	stream << ".MOLE."; break;
		case ENUM_NEWTON:	stream << ".NEWTON."; break;
		case ENUM_OHM:	stream << ".OHM."; break;
		case ENUM_PASCAL:	stream << ".PASCAL."; break;
		case ENUM_RADIAN:	stream << ".RADIAN."; break;
		case ENUM_SECOND:	stream << ".SECOND."; break;
		case ENUM_SIEMENS:	stream << ".SIEMENS."; break;
		case ENUM_SIEVERT:	stream << ".SIEVERT."; break;
		case ENUM_SQUARE_METRE:	stream << ".SQUARE_METRE."; break;
		case ENUM_STERADIAN:	stream << ".STERADIAN."; break;
		case ENUM_TESLA:	stream << ".TESLA."; break;
		case ENUM_VOLT:	stream << ".VOLT."; break;
		case ENUM_WATT:	stream << ".WATT."; break;
		case ENUM_WEBER:	stream << ".WEBER."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcSIUnitName> IFC4X3::IfcSIUnitName::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcSIUnitName>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcSIUnitName>(); }
	shared_ptr<IfcSIUnitName> type_object( new IfcSIUnitName() );
	if( std_iequal( arg, ".AMPERE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_AMPERE;
	}
	else if( std_iequal( arg, ".BECQUEREL." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_BECQUEREL;
	}
	else if( std_iequal( arg, ".CANDELA." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_CANDELA;
	}
	else if( std_iequal( arg, ".COULOMB." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_COULOMB;
	}
	else if( std_iequal( arg, ".CUBIC_METRE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_CUBIC_METRE;
	}
	else if( std_iequal( arg, ".DEGREE_CELSIUS." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_DEGREE_CELSIUS;
	}
	else if( std_iequal( arg, ".FARAD." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_FARAD;
	}
	else if( std_iequal( arg, ".GRAM." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_GRAM;
	}
	else if( std_iequal( arg, ".GRAY." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_GRAY;
	}
	else if( std_iequal( arg, ".HENRY." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_HENRY;
	}
	else if( std_iequal( arg, ".HERTZ." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_HERTZ;
	}
	else if( std_iequal( arg, ".JOULE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_JOULE;
	}
	else if( std_iequal( arg, ".KELVIN." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_KELVIN;
	}
	else if( std_iequal( arg, ".LUMEN." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_LUMEN;
	}
	else if( std_iequal( arg, ".LUX." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_LUX;
	}
	else if( std_iequal( arg, ".METRE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_METRE;
	}
	else if( std_iequal( arg, ".MOLE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_MOLE;
	}
	else if( std_iequal( arg, ".NEWTON." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_NEWTON;
	}
	else if( std_iequal( arg, ".OHM." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_OHM;
	}
	else if( std_iequal( arg, ".PASCAL." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_PASCAL;
	}
	else if( std_iequal( arg, ".RADIAN." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_RADIAN;
	}
	else if( std_iequal( arg, ".SECOND." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_SECOND;
	}
	else if( std_iequal( arg, ".SIEMENS." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_SIEMENS;
	}
	else if( std_iequal( arg, ".SIEVERT." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_SIEVERT;
	}
	else if( std_iequal( arg, ".SQUARE_METRE." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_SQUARE_METRE;
	}
	else if( std_iequal( arg, ".STERADIAN." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_STERADIAN;
	}
	else if( std_iequal( arg, ".TESLA." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_TESLA;
	}
	else if( std_iequal( arg, ".VOLT." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_VOLT;
	}
	else if( std_iequal( arg, ".WATT." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_WATT;
	}
	else if( std_iequal( arg, ".WEBER." ) )
	{
		type_object->m_enum = IfcSIUnitName::ENUM_WEBER;
	}
	return type_object;
}
