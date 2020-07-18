class OdomDouble {
	private:
		double x, y, z;
		double o_x, o_y, o_z, o_w;

	public:
		OdomDouble(double x, double y, double z, double o_x, double o_y, double o_z, double o_w) {
			this->x = x;
			this->y = y;
			this->z = 0.0;
			this->o_x = o_x;
			this->o_y = o_y;
			this->o_z = o_z;
			this->o_w = o_w;
		}

		// getters
		double getX() {
			return this->x;
		}

		double getY() {
			return this->y;
		}

		double getZ() {
			return this->z;
		}

		double getOX() {
			return this->o_x;
		}

		double getOY() {
			return this->o_y;
		}

		double getOZ() {
			return this->o_z;
		}

		double getOW() {
			return this->o_w;
		}
};
