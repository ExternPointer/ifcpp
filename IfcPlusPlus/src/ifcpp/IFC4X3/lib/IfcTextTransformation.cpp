/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcTextTransformation.h"

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
