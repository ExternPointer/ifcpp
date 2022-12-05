/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcCompositeCurve.h"
#include "ifcpp/IFC4X3/include/IfcPresentationLayerAssignment.h"
#include "ifcpp/IFC4X3/include/IfcSegment.h"
#include "ifcpp/IFC4X3/include/IfcStyledItem.h"
#include "ifcpp/IFC4X3/include/IfcTransitionCode.h"

// ENTITY IfcSegment 
IFC4X3::IfcSegment::IfcSegment( int tag ) { m_tag = tag; }
void IFC4X3::IfcSegment::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSEGMENT" << "(";
	if( m_Transition ) { m_Transition->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSegment::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSegment::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 1 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSegment, expecting 1, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Transition = IfcTransitionCode::createObjectFromSTEP( args[0], map, errorStream );
}
void IFC4X3::IfcSegment::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Transition", m_Transition ) );
}
void IFC4X3::IfcSegment::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributesInverse( vec_attributes_inverse );
	if( !m_UsingCurves_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> UsingCurves_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_UsingCurves_inverse.size(); ++i )
		{
			if( !m_UsingCurves_inverse[i].expired() )
			{
				UsingCurves_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcCompositeCurve>( m_UsingCurves_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "UsingCurves_inverse", UsingCurves_inverse_vec_obj ) );
	}
}
void IFC4X3::IfcSegment::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcGeometricRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSegment::unlinkFromInverseCounterparts()
{
	IfcGeometricRepresentationItem::unlinkFromInverseCounterparts();
}
