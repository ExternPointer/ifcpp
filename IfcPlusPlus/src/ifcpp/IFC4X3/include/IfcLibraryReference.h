/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcLibrarySelect.h"
#include "IfcExternalReference.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcText;
	class IFCQUERY_EXPORT IfcLanguageId;
	class IFCQUERY_EXPORT IfcLibraryInformation;
	class IFCQUERY_EXPORT IfcRelAssociatesLibrary;
	//ENTITY
	class IFCQUERY_EXPORT IfcLibraryReference : virtual public IfcLibrarySelect, public IfcExternalReference
	{ 
	public:
		IfcLibraryReference() = default;
		IfcLibraryReference( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 6; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 3452421091; }

		// IfcExternalReference -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcURIReference>									m_Location;					//optional
		//  shared_ptr<IfcIdentifier>									m_Identification;			//optional
		//  shared_ptr<IfcLabel>										m_Name;						//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_ExternalReferenceForResources_inverse;

		// IfcLibraryReference -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcText>											m_Description;				//optional
		shared_ptr<IfcLanguageId>									m_Language;					//optional
		shared_ptr<IfcLibraryInformation>							m_ReferencedLibrary;		//optional
		// inverse attributes:
		std::vector<weak_ptr<IfcRelAssociatesLibrary> >				m_LibraryRefForObjects_inverse;
	};
}

