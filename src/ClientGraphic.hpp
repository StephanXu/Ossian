#ifndef OSSIAN_CLIENT_GRAPHIC
#define OSSIAN_CLIENT_GRAPHIC

#include <ossian/Factory.hpp>

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <array>

#include "Referee.hpp"

//[ATTENTION]: We didn't check any range of arguments

class IClientGraphicElementStyle
{
public:
	virtual auto FillGraphicData(GraphicData& graphicData) const -> void = 0;
};

class LineStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	uint16_t m_EndX;
	uint16_t m_EndY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 0;
		graphicData.m_Color       = m_Color;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_StartX;
		graphicData.m_StartY      = m_StartY;
		graphicData.m_EndX        = m_EndX;
		graphicData.m_EndY        = m_EndY;
	}
};

class RectangleStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	uint16_t m_EndX;
	uint16_t m_EndY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 1;
		graphicData.m_Color       = m_Color;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_StartX;
		graphicData.m_StartY      = m_StartY;
		graphicData.m_EndX        = m_EndX;
		graphicData.m_EndY        = m_EndY;
	}
};

class CircleStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_Radius;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 2;
		graphicData.m_Color       = m_Color;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_CenterX;
		graphicData.m_StartY      = m_CenterY;
		graphicData.m_Radius      = m_Radius;
	}
};

class EllipseStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_HalfAxisX;
	uint16_t m_HalfAxisY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 3;
		graphicData.m_Color       = m_Color;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_CenterX;
		graphicData.m_StartY      = m_CenterY;
		graphicData.m_EndX        = m_HalfAxisX;
		graphicData.m_EndY        = m_HalfAxisY;
	}
};

class ArcStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_StartAngle;
	uint16_t m_EndAngle;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_HalfAxisX;
	uint16_t m_HalfAxisY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 4;
		graphicData.m_Color       = m_Color;
		graphicData.m_StartAngle  = m_StartAngle;
		graphicData.m_EndAngle    = m_EndAngle;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_CenterX;
		graphicData.m_StartY      = m_CenterY;
		graphicData.m_EndX        = m_HalfAxisX;
		graphicData.m_EndY        = m_HalfAxisY;
	}
};

class TextStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_FontSize;
	uint16_t m_TextLength;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;

	std::string m_Text;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 7;
		graphicData.m_Color       = m_Color;
		graphicData.m_StartAngle  = m_FontSize;
		graphicData.m_EndAngle    = m_TextLength;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_StartX;
		graphicData.m_StartY      = m_StartY;
	}
};

class IClientGraphicElement
{
public:
	virtual ~IClientGraphicElement() = default;
	virtual auto FillGraphicData(GraphicData& graphicData) -> void = 0;
	virtual auto IsText() const -> bool = 0;
	virtual auto IsModified() const -> bool = 0;
};

template <typename StyleType>
class ClientGraphicElement : public IClientGraphicElement
{
public:
	explicit ClientGraphicElement(const std::array<uint8_t, 3> name,
	                              const uint8_t layer)
		: m_Layer(layer), m_Style()
	{
		std::copy(name.begin(), name.end(), m_GraphicName);
	}

	virtual ~ClientGraphicElement() = default;

	auto FillGraphicData(GraphicData& graphicData) -> void override
	{
		std::copy(m_GraphicName, m_GraphicName + 3, graphicData.m_GraphicName);
		if (m_IsWaitDelete)
		{
			graphicData.m_OperateType = 3;
			m_IsWaitDelete            = false;
			m_IsInitialized           = false;
		}
		else if (!m_IsInitialized)
		{
			graphicData.m_OperateType = 1;
			m_IsInitialized           = true;
		}
		else
		{
			graphicData.m_OperateType = 2;
		}
		graphicData.m_Layer = m_Layer;
		m_Style.FillGraphicData(graphicData);
		m_IsModified = false;
	}

	auto GetStyleRef() -> StyleType&
	{
		return m_Style;
	}

	auto Save() -> void
	{
		m_IsModified = true;
	}

	auto Delete() -> void
	{
		m_IsWaitDelete = true;
		m_IsModified   = true;
	}

	auto Layer() const -> uint8_t { return m_Layer; }

	auto IsText() const -> bool override { return std::is_same_v<StyleType, TextStyle>; }

	auto IsModified() const -> bool override { return m_IsModified; }

private:
	uint8_t m_GraphicName[3] = {0, 0, 0};
	uint8_t m_Layer;

	StyleType m_Style;

	bool m_IsInitialized = false;
	bool m_IsModified    = true;
	bool m_IsWaitDelete  = false;
};

class ClientGraphic
{
public:
	explicit ClientGraphic(const uint16_t id, const IReferee* referee)
		: m_Id(id),
		  m_Referee(referee)
	{
	}

	template <typename StyleType>
	auto AddElement(const uint8_t layer) -> std::shared_ptr<ClientGraphicElement<StyleType>>
	{
		m_Elements.push_back(std::make_shared<ClientGraphicElement<StyleType>>(GetNextGraphicName(), layer));
		return std::dynamic_pointer_cast<ClientGraphicElement<StyleType>>(m_Elements.back());
	}

	auto Render(const bool repaint = false) -> void
	{
		for (auto&& element : m_Elements)
		{
			if (element->IsModified() || repaint)
			{
				if (element->IsText())
				{
					ClientGraphicTextModify buffer{};
					buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
					buffer.m_Header.m_ReceiverID = m_Id;
					buffer.m_Header.m_SenderID   = m_Referee->Id();
					element->FillGraphicData(buffer.m_GraphicData);
					m_Referee->SendMessage(RefereeBuffer<ClientGraphicTextModify>(buffer));
				}
				else
				{
					ClientGraphicShapeModify<1> buffer{};
					buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
					buffer.m_Header.m_ReceiverID = m_Id;
					buffer.m_Header.m_SenderID   = m_Referee->Id();
					element->FillGraphicData(buffer.m_GraphicData[0]);
					m_Referee->SendMessage(RefereeBuffer<ClientGraphicShapeModify<1>>(buffer));
				}
			}
		}
	}

private:

	auto GetNextGraphicName() -> std::array<uint8_t, 3>
	{
		for (size_t i = 0; i < m_Name.size(); ++i)
		{
			if (++m_Name[i] <= '9')
			{
				return m_Name;
			}
			m_Name[i] = '0';
		}
		throw std::bad_alloc(); //[TODO]: Would better define a new exception type.
	}

	uint16_t m_Id = 0;
	std::vector<std::shared_ptr<IClientGraphicElement>> m_Elements{};
	std::array<uint8_t, 3> m_Name{'0', '0', '0'};
	const IReferee* m_Referee;
};

class ClientGraphicManager
{
public:
	OSSIAN_SERVICE_SETUP(ClientGraphicManager(IReferee* referee))
		: m_Referee(referee)
	{
	}

	auto AddOrGetGraphicClient(uint16_t id) -> std::shared_ptr<ClientGraphic>
	{
		const auto it = m_Client.find(id);
		if (it != m_Client.end())
		{
			return it->second;
		}
		m_Client.insert(std::make_pair(id, std::make_shared<ClientGraphic>(id, m_Referee)));
		return m_Client[id];
	}

private:
	std::unordered_map<uint16_t, std::shared_ptr<ClientGraphic>> m_Client;
	IReferee* m_Referee;
};


#endif // OSSIAN_CLIENT_GRAPHIC
