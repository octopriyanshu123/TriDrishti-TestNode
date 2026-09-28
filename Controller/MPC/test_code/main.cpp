#include <iostreams>

class SystemModel
{
private:
public:
    SystemModel(/* args */);
    ~SystemModel();

    int get_state_size()
    {
    }

    int get_control_size()
    {
    }

    void dynamics()
    {
    }

    void get_control_bounds()
    {
    }

    void get_state_bounds()
    {
    }
};

class MPC_Controller
{
    public:
    MPC_Controller(){

    }

    ~MPC_Controller(){


    }

    void set_weights(double R, double Q){

    }

    void compute_control()

    void _generate_control_candidates(){

    }

    void _evaluate_control(){

    }

    void _compute_constraint_penalty(){

    }


};

