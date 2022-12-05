/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcTextFontSelect.h"
#include "IfcExternalReference.h"
namespace IFC4X3
{
	//ENTITY
	class IFCQUERY_EXPORT IfcExternallyDefinedTextFont : virtual public IfcTextFontSelect, public IfcExternalReference
	{ 
	public:
		IfcExternallyDefinedTextFont() = default;
		IfcExternallyDefinedTextFont( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 3548104201; }

		// IfcExternalReference -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcURIReference>									m_Location;					//optional
		//  shared_ptr<IfcIdentifier>									m_Identification;			//optional
		//  shared_ptr<IfcLabel>										m_Name;						//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_ExternalReferenceForResources_inverse;

		// IfcExternallyDefinedTextFont -----------------------------------------------------------
	};
}

