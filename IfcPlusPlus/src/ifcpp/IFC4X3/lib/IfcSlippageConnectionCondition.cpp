/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcLabel.h"
#include "ifcpp/IFC4X3/include/IfcLengthMeasure.h"
#include "ifcpp/IFC4X3/include/IfcSlippageConnectionCondition.h"

// ENTITY IfcSlippageConnectionCondition 
IFC4X3::IfcSlippageConnectionCondition::IfcSlippageConnectionCondition( int tag ) { m_tag = tag; }
void IFC4X3::IfcSlippageConnectionCondition::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSLIPPAGECONNECTIONCONDITION" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SlippageX ) { m_SlippageX->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SlippageY ) { m_SlippageY->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SlippageZ ) { m_SlippageZ->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSlippageConnectionCondition::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSlippageConnectionCondition::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSlippageConnectionCondition, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_SlippageX = IfcLengthMeasure::createObjectFromSTEP( args[1], map, errorStream );
	m_SlippageY = IfcLengthMeasure::createObjectFromSTEP( args[2], map, errorStream );
	m_SlippageZ = IfcLengthMeasure::createObjectFromSTEP( args[3], map, errorStream );
}
void IFC4X3::IfcSlippageConnectionCondition::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcStructuralConnectionCondition::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "SlippageX", m_SlippageX ) );
	vec_attributes.emplace_back( std::make_pair( "SlippageY", m_SlippageY ) );
	vec_attributes.emplace_back( std::make_pair( "SlippageZ", m_SlippageZ ) );
}
void IFC4X3::IfcSlippageConnectionCondition::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcStructuralConnectionCondition::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcSlippageConnectionCondition::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcStructuralConnectionCondition::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSlippageConnectionCondition::unlinkFromInverseCounterparts()
{
	IfcStructuralConnectionCondition::unlinkFromInverseCounterparts();
}
