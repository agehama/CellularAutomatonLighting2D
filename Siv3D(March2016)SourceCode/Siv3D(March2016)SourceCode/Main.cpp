/**
CellularAutomatonLighting2D

Copyright (c) 2016 agehama

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#include <array>
#include <Siv3D.hpp>

template<class T>
class DoubleBuffer
{
public:

	DoubleBuffer() {}

	DoubleBuffer(const T& initial) :m_buffer({ initial, initial }) {}

	void flip()
	{
		m_currentWriteBuffer = (m_currentWriteBuffer + 1) % m_buffer.size();
	}

	T& write()
	{
		return m_buffer[writeIndex()];
	}

	const T& read()const
	{
		return m_buffer[readIndex()];
	}

private:

	int writeIndex()const
	{
		return m_currentWriteBuffer;
	}

	int readIndex()const
	{
		return (m_currentWriteBuffer + 1) % m_buffer.size();
	}

	std::array<T, 2> m_buffer;
	int m_currentWriteBuffer = 0;
};

template<class T>
class Grid2D
{
public:

	Grid2D() {}

	Grid2D(size_t x, size_t y)
		:m_grid(ColumnType(y, std::vector<T>(x)))
	{}

	Grid2D(size_t x, size_t y, const T& value)
		:m_grid(ColumnType(y, std::vector<T>(x, value)))
	{}

	void resize(size_t x, size_t y)
	{
		m_grid.resize(y);
		for (auto& line : m_grid)
		{
			line.resize(x);
		}
	}

	void resize(size_t x, size_t y, const T& value)
	{
		m_grid.resize(y);
		for (auto& line : m_grid)
		{
			line.resize(x, value);
		}
	}

	void reset(const T& value)
	{
		for (auto& line : m_grid)
		{
			for (auto& elem : line)
			{
				elem = value;
			}
		}
	}

	std::vector<T>& operator[](size_t y)
	{
		return m_grid[y];
	}

	const std::vector<T>& operator[](size_t y)const
	{
		return m_grid[y];
	}

	T& operator[](const Point& p)
	{
		return m_grid[p.y][p.x];
	}

	const T& operator[](const Point& p)const
	{
		return m_grid[p.y][p.x];
	}

	bool isValid(const Point& p)const
	{
		return 0 <= p.y && p.y < static_cast<int>(m_grid.size())
			&& 0 <= p.x && p.x < static_cast<int>(m_grid[p.y].size());
	}

	size_t width()const
	{
		return m_grid.empty() ? 0u : m_grid.front().size();
	}

	size_t height()const
	{
		return m_grid.size();
	}

private:

	using RowType = std::vector<T>;
	using ColumnType = std::vector<RowType>;

	std::vector<std::vector<T>> m_grid;
};

class Field
{
public:

	Field(const Image& image = Image(Window::Size(), Palette::White), int gridUnitPixel = 32)
		: m_field(image)
		, m_texture(image)
		, m_isWall(m_field.width / gridUnitPixel, m_field.height / gridUnitPixel, FieldSpace())
		, m_brightness(Grid2D<ColorF>(m_field.width / gridUnitPixel, m_field.height / gridUnitPixel, Palette::Black))
	{
		checkInitialValidness(gridUnitPixel);
		init();
	}

	void update()
	{
		resetBrightness();

		const auto mousePos = mouseGridPos();
		if (m_isWall.isValid(mousePos))
		{
			if (Input::MouseL.pressed)
			{
				m_isWall[mousePos] = FieldWall();
			}
			if (Input::MouseR.pressed)
			{
				m_isWall[mousePos] = FieldSpace();
			}
		}

		const auto field = fieldRect();

		const double dt = 1.0 / 60.0;

		const double restitution = 0.5;
		const std::array<Point, 8> neighbors =
		{
			Point(+0,-1),Point(-1,+0),Point(+1,+0),Point(+0,+1),
			Point(-1,-1),Point(+1,-1),Point(-1,+1),Point(+1,+1)
		};
		const std::array<Vec2, 8> reflectDirection =
		{
			Vec2(+1,-restitution),Vec2(-restitution,+1),Vec2(-restitution,+1),Vec2(+1,-restitution),
			Vec2(-restitution,-restitution),Vec2(-restitution,-restitution),Vec2(-restitution,-restitution),Vec2(-restitution,-restitution)
		};

		for (size_t i = 0; i < m_lightPos.size(); ++i)
		{
			//減衰力
			//damping force
			m_velocity[i] *= 0.999;

			const Vec2 toMouse = Mouse::PosF() - m_lightPos[i].center;
			if (Input::KeySpace.pressed)
			{
				if (Input::MouseM.pressed)
				{
					m_velocity[i] += toMouse*0.5*dt;
				}
				else if (1.0 < toMouse.lengthSq())
				{
					m_velocity[i] += -toMouse / toMouse.lengthSq()*10000.0*dt;
				}
			}
			else
			{
				m_velocity[i] += RandomVec2(1000.0)*dt;
			}

			const Line moveSegment(m_lightPos[i].center, m_lightPos[i].center + m_velocity[i] * dt);
			const Point gridA = gridPos(m_lightPos[i].center.asPoint());
			const Point gridB = gridPos((m_lightPos[i].center + m_velocity[i] * dt).asPoint());

			//ライトと壁の衝突判定
			//Collision detection between lights and walls.
			if (
				//範囲外参照を避けるためフィールド内のみ考慮する
				//To avoid outrange reference, only considering inner field.
				m_isWall.isValid(gridA) && m_isWall.isValid(gridB)

				//衝突はライトがグリッド境界を跨ぐときのみ起こる
				//Collision may occur when a light strides over grid boundary.
				&& gridA != gridB

				//ライトが既に壁に埋まっているときは、まず外に出ることを優先する
				//If a light is already buried in wall, then give priority to going outside.
				&& !isWall(gridA)
				)
			{
				bool reflects = false;
				for (size_t j = 0; j < neighbors.size(); ++j)
				{
					//壁をすり抜けない　かつ　壁に沿って滑れるように
					//To avoid passing through in wall while enable sliding across wall.
					if (reflects && 4 <= j)
					{
						break;
					}

					if (m_isWall.isValid(gridA + neighbors[j]) && isWall(gridA + neighbors[j]) && RectF(gridRect(gridA + neighbors[j])).stretched(2.0).intersects(moveSegment))
					{
						const Vec2 scale = reflectDirection[j];
						m_velocity[i].x *= scale.x;
						m_velocity[i].y *= scale.y;
						reflects = true;
					}
				}
			}

			m_lightPos[i].center += m_velocity[i] * dt;

			const auto pos = gridPos(m_lightPos[i].center.asPoint());
			if (m_brightness.read().isValid(pos))
			{
				m_brightness.write()[pos] = m_lightColor[i];
			}
		}

		m_brightness.flip();

		for (int i = 0; i < 30; ++i)
		{
			stepLightDiffusion();
		}
	}

	void draw()const
	{
		for (size_t y = 0; y < m_isWall.height(); ++y)
		{
			for (size_t x = 0; x < m_isWall.width(); ++x)
			{
				const auto color = m_brightness.read()[{x, y}];

				const Rect rect = gridRect({ x, y });

				if (isWall({ x, y }))
				{
					rect.draw(Palette::Black);
				}
				else
				{
					m_texture.uv(rect).draw(rect.pos, color);
				}
			}
		}

		for (size_t i = 0; i < m_lightPos.size(); ++i)
		{
			m_lightPos[i].draw(m_lightColor[i]);
		}
	}

	static char FieldWall()
	{
		return static_cast<char>(true);
	}

	static char FieldSpace()
	{
		return static_cast<char>(false);
	}

private:

	void checkInitialValidness(int gridUnitPixel)const
	{
		const bool condition = m_field.width % gridUnitPixel == 0 && m_field.height % gridUnitPixel == 0;
		if (!condition)
		{
			LOG_ERROR(L"Field Class Initialization Failed : Field Resolution Cannot Be Divided By Cell Unit Size.");
			assert(false);
		}
	}

	void init()
	{
		for (size_t y = 0; y < m_isWall.height(); ++y)
		{
			for (size_t x = 0; x < m_isWall.width(); ++x)
			{
				m_isWall[y][x] = FieldSpace();
				if (x == 0 || y == 0 || x + 1 == m_isWall.width() || y + 1 == m_isWall.height())
				{
					m_isWall[y][x] = FieldWall();
				}

				//ランダムに壁を配置する
				//Put blocks randomly.
				//m_isWall[y][x] = RandomBool(0.3) ? FieldWall() : FieldSpace();
			}
		}

		const int num = 8;
		m_lightPos.resize(num);
		m_lightColor.resize(num);
		m_velocity.resize(num);
		for (size_t i = 0; i < m_lightPos.size(); ++i)
		{
			m_lightPos[i] = Circle(RandomVec2(RectF(0, 0, Window::Size()).stretched(-gridUnitPixel())), gridUnitPixel()*0.5);
			m_lightColor[i] = HSV(120.0 + 30.0*i, 0.7, 1.0);
			m_velocity[i] = Vec2(0, 0);
		}
	}

	int gridUnitPixel()const
	{
		return m_field.height / m_isWall.height();
	}

	Rect gridRect(const Point& p)const
	{
		const int unitWidth = gridUnitPixel();
		return Rect(unitWidth*p.x, unitWidth*p.y, unitWidth, unitWidth);
	}

	Rect fieldRect()const
	{
		return Rect(0, 0, m_field.width, m_field.height);
	}

	bool isWall(const Point& p)const
	{
		return m_isWall[p.y][p.x] == FieldWall();
	}

	void resetBrightness()
	{
		m_brightness.write().reset(Palette::Black);
		m_brightness.flip();
		m_brightness.write().reset(Palette::Black);
	}

	Point mouseGridPos()const
	{
		const auto p = Mouse::Pos();
		return Point(p.x / gridUnitPixel(), p.y / gridUnitPixel());
	}

	Point gridPos(const Point& p)const
	{
		return Point(Floor(1.0*p.x / gridUnitPixel()), Floor(1.0*p.y / gridUnitPixel()));
	}

	void stepLightDiffusion()
	{
		const double sqrt2 = Sqrt(2.0);

		std::array<Point, 8> neighbors =
		{
			Point(-1,-1),Point(+0,-1),Point(+1,-1),
			Point(-1,+0),             Point(+1,+0),
			Point(-1,+1),Point(+0,+1),Point(+1,+1)
		};

		const double attenuationAdjacent = 0.9;
		const double attenuationDiagonal = pow(attenuationAdjacent, sqrt2);
		const std::array<double, 8> attenuations =
		{
			attenuationDiagonal,attenuationAdjacent,attenuationDiagonal,
			attenuationAdjacent,                    attenuationAdjacent,
			attenuationDiagonal,attenuationAdjacent,attenuationDiagonal
		};

		for (size_t y = 0; y < m_brightness.read().height(); ++y)
		{
			for (size_t x = 0; x < m_brightness.read().width(); ++x)
			{
				if (isWall({ x, y }))
				{
					m_brightness.write()[{x, y}] = Palette::Black;
					continue;
				}

				ColorF maxBrightness = Palette::Black;
				for (size_t i = 0; i < neighbors.size(); ++i)
				{
					//縦横どちらかがつながっていないと斜め方向に光は届かない
					//Light isn't propagate diagonally in case that blocks are put length and width.
					if (
						(i == 0 || i == 2 || i == 5 || i == 7)
						&& isWall({ x + neighbors[i].x, y })
						&& isWall({ x, y + neighbors[i].y })
						)
					{
						continue;
					}

					const double a = attenuations[i];
					const auto& side = neighbors[i];
					const Point sideCell = Point(x, y) + side;
					if (m_brightness.read().isValid(sideCell))
					{
						maxBrightness.r = Max(maxBrightness.r, m_brightness.read()[sideCell].r*a);
						maxBrightness.g = Max(maxBrightness.g, m_brightness.read()[sideCell].g*a);
						maxBrightness.b = Max(maxBrightness.b, m_brightness.read()[sideCell].b*a);
					}
				}

				m_brightness.write()[{x, y}].r = Max(m_brightness.read()[{x, y}].r, maxBrightness.r);
				m_brightness.write()[{x, y}].g = Max(m_brightness.read()[{x, y}].g, maxBrightness.g);
				m_brightness.write()[{x, y}].b = Max(m_brightness.read()[{x, y}].b, maxBrightness.b);
			}
		}

		m_brightness.flip();
	}

	Image m_field;
	Texture m_texture;
	Grid2D<char> m_isWall;
	DoubleBuffer<Grid2D<ColorF>> m_brightness;

	std::vector<Circle> m_lightPos;
	std::vector<ColorF> m_lightColor;
	std::vector<Vec2> m_velocity;
};

void Main()
{
	Window::Resize(1280, 736);
	Field field(Image(Window::Size(), Palette::White), 32);

	while (System::Update())
	{
		field.update();
		field.draw();

		Window::SetTitle(Profiler::FPS());
	}
}
