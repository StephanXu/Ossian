#ifndef OSSIAN_CLIENT_GRAPHIC
#define OSSIAN_CLIENT_GRAPHIC

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


class ClientGraphicElement
{
public:
	explicit ClientGraphicElement(const std::string name,
	                              const uint8_t layer,
	                              IClientGraphicElementStyle& style)
		: m_GraphicName{0, 0, 0}
		  , m_Layer(layer)
		  , m_Style(style)
		  , m_IsInitialized(false)
	{
		std::copy(name.begin(),
		          name.begin() + std::min(3, static_cast<int>(name.length())),
		          m_GraphicName);
	}

	auto FillGraphicData(GraphicData& graphicData) const -> void
	{
		std::copy(m_GraphicName, m_GraphicName + 3, graphicData.m_GraphicName);
		graphicData.m_OperateType = m_IsInitialized ? 2 : 1;
		m_Style.FillGraphicData(graphicData);
	}

	auto Layer() const -> uint8_t { return m_Layer; }

private:
	uint8_t m_GraphicName[3];
	uint8_t m_Layer;

	IClientGraphicElementStyle& m_Style;

	bool m_IsInitialized;
};


#endif // OSSIAN_CLIENT_GRAPHIC
