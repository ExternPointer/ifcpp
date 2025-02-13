/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcBSplineSurfaceWithKnots.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcReal;
	//ENTITY
	class IFCQUERY_EXPORT IfcRationalBSplineSurfaceWithKnots : public IfcBSplineSurfaceWithKnots
	{
	public:
		IfcRationalBSplineSurfaceWithKnots() = default;
		IfcRationalBSplineSurfaceWithKnots( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 13; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 683857671; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcGeometricRepresentationItem -----------------------------------------------------------

		// IfcSurface -----------------------------------------------------------

		// IfcBoundedSurface -----------------------------------------------------------

		// IfcBSplineSurface -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcInteger>									m_UDegree;
		//  shared_ptr<IfcInteger>									m_VDegree;
		//  std::vector<std::vector<shared_ptr<IfcCartesianPoint> > >	m_ControlPointsList;
		//  shared_ptr<IfcBSplineSurfaceForm>						m_SurfaceForm;
		//  shared_ptr<IfcLogical>									m_UClosed;
		//  shared_ptr<IfcLogical>									m_VClosed;
		//  shared_ptr<IfcLogical>									m_SelfIntersect;

		// IfcBSplineSurfaceWithKnots -----------------------------------------------------------
		// attributes:
		//  std::vector<shared_ptr<IfcInteger> >					m_UMultiplicities;
		//  std::vector<shared_ptr<IfcInteger> >					m_VMultiplicities;
		//  std::vector<shared_ptr<IfcParameterValue> >				m_UKnots;
		//  std::vector<shared_ptr<IfcParameterValue> >				m_VKnots;
		//  shared_ptr<IfcKnotType>									m_KnotSpec;

		// IfcRationalBSplineSurfaceWithKnots -----------------------------------------------------------
		// attributes:
		std::vector<std::vector<shared_ptr<IfcReal> > >			m_WeightsData;
	};
}
