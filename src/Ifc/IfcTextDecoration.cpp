/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcTextDecoration.h"

// TYPE IfcTextDecoration = STRING;
IFC4X3::IfcTextDecoration::IfcTextDecoration( std::string value ) { m_value = value; }
void IFC4X3::IfcTextDecoration::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCTEXTDECORATION("; }
	stream << "'" << encodeStepString( m_value ) << "'";
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcTextDecoration> IFC4X3::IfcTextDecoration::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcTextDecoration>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcTextDecoration>(); }
	shared_ptr<IfcTextDecoration> type_object( new IfcTextDecoration() );
	readString( arg, type_object->m_value );
	return type_object;
}
