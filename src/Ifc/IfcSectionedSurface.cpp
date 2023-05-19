/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2PlacementLinear.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcSectionedSurface.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcSectionedSurface 
IFC4X3::IfcSectionedSurface::IfcSectionedSurface( int tag ) { m_tag = tag; }
void IFC4X3::IfcSectionedSurface::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSECTIONEDSURFACE" << "(";
	if( m_Directrix ) { stream << "#" << m_Directrix->m_tag; } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_CrossSectionPositions );
	stream << ",";
	writeEntityList( stream, m_CrossSections );
	stream << ");";
}
void IFC4X3::IfcSectionedSurface::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSectionedSurface::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSectionedSurface, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_Directrix, map, errorStream );
	readEntityReferenceList( args[1], m_CrossSectionPositions, map, errorStream );
	readEntityReferenceList( args[2], m_CrossSections, map, errorStream );
}
void IFC4X3::IfcSectionedSurface::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSurface::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Directrix", m_Directrix ) );
	shared_ptr<AttributeObjectVector> CrossSectionPositions_vec_object( new AttributeObjectVector() );
	std::copy( m_CrossSectionPositions.begin(), m_CrossSectionPositions.end(), std::back_inserter( CrossSectionPositions_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "CrossSectionPositions", CrossSectionPositions_vec_object ) );
	shared_ptr<AttributeObjectVector> CrossSections_vec_object( new AttributeObjectVector() );
	std::copy( m_CrossSections.begin(), m_CrossSections.end(), std::back_inserter( CrossSections_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "CrossSections", CrossSections_vec_object ) );
}
void IFC4X3::IfcSectionedSurface::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSurface::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcSectionedSurface::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSurface::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSectionedSurface::unlinkFromInverseCounterparts()
{
	IfcSurface::unlinkFromInverseCounterparts();
}
