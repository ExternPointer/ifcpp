/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcCardinalPointReference.h"

// TYPE IfcCardinalPointReference = INTEGER;
IFC4X3::IfcCardinalPointReference::IfcCardinalPointReference( int value ) { m_value = value; }
void IFC4X3::IfcCardinalPointReference::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCARDINALPOINTREFERENCE("; }
	stream << m_value;
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcCardinalPointReference> IFC4X3::IfcCardinalPointReference::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcCardinalPointReference>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcCardinalPointReference>(); }
	shared_ptr<IfcCardinalPointReference> type_object( new IfcCardinalPointReference() );
	readInteger( arg, type_object->m_value );
	return type_object;
}