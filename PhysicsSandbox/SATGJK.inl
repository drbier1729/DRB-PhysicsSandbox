
namespace drb::physics::util {
	inline Simplex::Type Simplex::GetType() const
	{
		return Type{ size };
	}
}