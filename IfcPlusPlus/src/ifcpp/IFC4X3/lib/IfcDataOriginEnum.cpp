/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcDataOriginEnum.h"

// TYPE IfcDataOriginEnum = ENUMERATION OF	(MEASURED	,PREDICTED	,SIMULATED	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcDataOriginEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCDATAORIGINENUM("; }
	switch( m_enum )
	{
		case ENUM_MEASURED:	stream << ".MEASURED."; break;
		case ENUM_PREDICTED:	stream << ".PREDICTED."; break;
		case ENUM_SIMULATED:	stream << ".SIMULATED."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcDataOriginEnum> IFC4X3::IfcDataOriginEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcDataOriginEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcDataOriginEnum>(); }
	shared_ptr<IfcDataOriginEnum> type_object( new IfcDataOriginEnum() );
	if( std_iequal( arg, ".MEASURED." ) )
	{
		type_object->m_enum = IfcDataOriginEnum::ENUM_MEASURED;
	}
	else if( std_iequal( arg, ".PREDICTED." ) )
	{
		type_object->m_enum = IfcDataOriginEnum::ENUM_PREDICTED;
	}
	else if( std_iequal( arg, ".SIMULATED." ) )
	{
		type_object->m_enum = IfcDataOriginEnum::ENUM_SIMULATED;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcDataOriginEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcDataOriginEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
