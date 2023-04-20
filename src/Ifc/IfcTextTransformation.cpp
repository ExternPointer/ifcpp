/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcTextTransformation.h"

// TYPE IfcTextTransformation = STRING;
IFC4X3::IfcTextTransformation::IfcTextTransformation( std::string value ) { m_value = value; }
void IFC4X3::IfcTextTransformation::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCTEXTTRANSFORMATION("; }
	stream << "'" << encodeStepString( m_value ) << "'";
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcTextTransformation> IFC4X3::IfcTextTransformation::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcTextTransformation>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcTextTransformation>(); }
	shared_ptr<IfcTextTransformation> type_object( new IfcTextTransformation() );
	readString( arg, type_object->m_value );
	return type_object;
}