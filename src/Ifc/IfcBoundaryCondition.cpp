/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoundaryCondition.h"
#include "ifcpp/Ifc/IfcLabel.h"

// ENTITY IfcBoundaryCondition 
IFC4X3::IfcBoundaryCondition::IfcBoundaryCondition( int tag ) { m_tag = tag; }
void IFC4X3::IfcBoundaryCondition::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCBOUNDARYCONDITION" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcBoundaryCondition::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcBoundaryCondition::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 1 ){ std::stringstream err; err << "Wrong parameter count for entity IfcBoundaryCondition, expecting 1, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
}
void IFC4X3::IfcBoundaryCondition::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
}
void IFC4X3::IfcBoundaryCondition::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
}
void IFC4X3::IfcBoundaryCondition::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcBoundaryCondition::unlinkFromInverseCounterparts()
{
}