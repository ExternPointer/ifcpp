/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcPreDefinedTextFont.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcTextFontName;
	class IFCQUERY_EXPORT IfcFontStyle;
	class IFCQUERY_EXPORT IfcFontVariant;
	class IFCQUERY_EXPORT IfcFontWeight;
	class IFCQUERY_EXPORT IfcSizeSelect;
	//ENTITY
	class IFCQUERY_EXPORT IfcTextStyleFontModel : public IfcPreDefinedTextFont
	{
	public:
		IfcTextStyleFontModel() = default;
		IfcTextStyleFontModel( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 6; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1983826977; }

		// IfcPresentationItem -----------------------------------------------------------

		// IfcPreDefinedItem -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>						m_Name;

		// IfcPreDefinedTextFont -----------------------------------------------------------

		// IfcTextStyleFontModel -----------------------------------------------------------
		// attributes:
		std::vector<shared_ptr<IfcTextFontName> >	m_FontFamily;
		shared_ptr<IfcFontStyle>					m_FontStyle;				//optional
		shared_ptr<IfcFontVariant>					m_FontVariant;				//optional
		shared_ptr<IfcFontWeight>					m_FontWeight;				//optional
		shared_ptr<IfcSizeSelect>					m_FontSize;
	};
}
