#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/trajectory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

class parabolicBlendWithViapoints
{

    private:
        rw::math::Q initialPose, endPose;
        std::vector<double> totalTime, blendTime, blendAcceleration;
        std::vector<rw::math::Q> viapoints;
        std::vector<rw::math::Q> acc, speed, pose, pathPoses, timesb, timesj, accumulatedTimej, accumulatedTimeb, xInitialb, xInitialj;

    public:

        rw::math::Q x(double t)
        {
            std::cout << "----------------------------- t = " << t << " -------------------------------------------" << std::endl;

            // Check the input time
            if (t > this->totalTime[this->totalTime.size()-1])
            {
                std::cout << " [WARNING] Introduced time is greater than the one introduced. Setting to output in the output...\n";
                t = this->totalTime[this->totalTime.size()-1];
            }

            if (t == this->totalTime[0])
            {
                return this->initialPose;
            }

            // Obtain the value itself
            rw::math::Q result(6,0.0);
            for (int i = 0; i < 6; i++)
            {
                int posj, posb;
                double timej, timeb;


                // Check time j
                for (uint j = 0; j < this->accumulatedTimej.size(); j++)
                {
                    if (t <= this->accumulatedTimej[j][i])
                    {
                        posj = j;
                        timej = this->accumulatedTimej[j][i];
                        break;
                    }
                }

                // Check time b
                for (uint j = 0; j < this->accumulatedTimeb.size(); j++)
                {
                    if (t <= this->accumulatedTimeb[j][i])
                    {
                        posb = j;
                        timeb = this->accumulatedTimeb[j][i];
                        break;
                    }
                }

                // Check which is the region in which it is and obtain the adequate value
                std::cout << " - The timeb and posb are: " << timeb << ", " << posb << std::endl;
                std::cout << " - The timej and posj are: " << timej << ", " << posj << std::endl;

                if (timej > timeb) // Region of constant speed
                {
                    result[i] = this->xInitialj[posj-1][i] + this->speed[posj-1][i]*(t - this->accumulatedTimej[posj][i]) ;
                }
                else // Region of acceleration
                {
                    if (posb == 1)
                    {
                        result[i] = this->xInitialb[posb-1][i] + 0.5*this->acc[posb-1][i]*pow(t - this->accumulatedTimeb[posb-1][i], 2);
                    }
                    else
                    {
                        result[i] = this->xInitialb[posb-1][i] + this->speed[posb-2][i]*(t - this->accumulatedTimeb[posb-1][i]) + 0.5*this->acc[posb-1][i]*pow(t - this->accumulatedTimeb[posb-1][i], 2);
                    }
                }
            }

            return result;
        }

    parabolicBlendWithViapoints(rw::math::Q input_initialPose, rw::math::Q input_endPose, std::vector<rw::math::Q> input_path, std::vector<double> input_blendAcceleration, std::vector<double> input_totalTime)
    {
        this->initialPose = input_initialPose;
        std::cout << "The initial pose is: "  << initialPose << std::endl;
        this->endPose = input_endPose;
        this->blendAcceleration = input_blendAcceleration;
        this->totalTime = input_totalTime;
        this->pathPoses = input_path;

        // Initialize other vectors to 0
        this->acc = std::vector<rw::math::Q>();
        this->speed = std::vector<rw::math::Q>();
        this->pose = std::vector<rw::math::Q>();
        this->timesb = std::vector<rw::math::Q>();
        this->timesj = std::vector<rw::math::Q>();
        this->accumulatedTimej = std::vector<rw::math::Q>();
        this->accumulatedTimeb = std::vector<rw::math::Q>();
        this->xInitialb = std::vector<rw::math::Q>();
        this->xInitialj = std::vector<rw::math::Q>();

        // Print points of the path
        for (uint i = 0; i <this->pathPoses.size(); i++)
        {
            std::cout << "  [NOTE] Path in position " << i << ": " << this->pathPoses[i] << std::endl;
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialization of vectors

        int n = this->pathPoses.size();
        for (int i = 0; i < n-1; i++)
        {
            this->timesj.push_back(rw::math::Q(6,0.0));
            this->xInitialj.push_back(rw::math::Q(6,0.0));
            this->speed.push_back(rw::math::Q(6,0.0));
        }

        for (int i = 0; i < n; i++)
        {
            this->timesb.push_back(rw::math::Q(6,0.0));
            this->accumulatedTimeb.push_back(rw::math::Q(6,0.0));
            this->accumulatedTimej.push_back(rw::math::Q(6,0.0));
            this->xInitialb.push_back(rw::math::Q(6,0.0));
            this->acc.push_back(rw::math::Q(6,0.0));
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Calculations for vectors
        // To-fill: timesb, timesj, speed(Q), accel(Q)

        std::cout << "-Number of speeds: " << this->speed.size() << std::endl;
        std::cout << "-Number of accelerations: " << this->acc.size() << std::endl;
        std::cout << "-Number of blend accelerations: " << this->blendAcceleration.size() << std::endl;

        /////////////////////////////////////
        // Calculate speeds
        for (int i = 1; i < n-2; i++) {
            this->speed[i] = (pathPoses[i+1] - pathPoses[i])/(this->totalTime[i+1] - this->totalTime[i]); }

        /////////////////////////////////////
        // Calculate accelerations
        for(int j = 0; j<6; j++){
            if ((this->pathPoses[1][j] - this->pathPoses[0][j]) == 0.0)
            {
                this->acc[0][j] = 0.0;    
            }
            else
            {
                this->acc[0][j]   = (this->pathPoses[1][j] - this->pathPoses[0][j])     >= 0 ? abs(this->blendAcceleration[0])   : -abs(this->blendAcceleration[0]);
            }
            
            if ((this->pathPoses[1][j] - this->pathPoses[0][j]) > 0)
            {
                this->acc[0][j] = abs(this->blendAcceleration[0]);
            }
            else if ((this->pathPoses[1][j] - this->pathPoses[0][j]) < 0)
            {
                this->acc[0][j] = -abs(this->blendAcceleration[0]);
            }
            else
            {
                this->acc[0][j] = 0;
            }

            if ((this->pathPoses[n-2][j] - this->pathPoses[n-1][j]) > 0)
            {
                this->acc[n-1][j] = abs(this->blendAcceleration[n-1]);
            }
            else if ((this->pathPoses[n-2][j] - this->pathPoses[n-1][j]) < 0)
            {
                this->acc[n-1][j] = -abs(this->blendAcceleration[n-1]);
            }
            else
            {
                this->acc[n-1][j] = 0;
            }
        }

        for (int i = 2; i < n-2; i++) {
            for(int j = 0; j<6; j++){
                if ((this->speed[i][j] - this->speed[i-1][j]) > 0)
                {
                    this->acc[i][j] = abs(this->blendAcceleration[i]);
                }
                else if ((this->speed[i][j] - this->speed[i-1][j]) < 0)
                {
                    this->acc[i][j] = -abs(this->blendAcceleration[i]);
                }
                else
                {
                    this->acc[i][j] = 0;
                }
            }
        }    
        
        /////////////////////////////////////
        // New parameters to be calculated
        for (int i = 0; i < 6; i++)
        {
            // Obtain timesb[0][i]
            if (this->acc[0][i] == 0)
            {
                this->timesb[0][i] = 0.0;
            }
            else
            {
                this->timesb[0][i] = (this->totalTime[1] - this->totalTime[0]) - sqrt(pow(this->totalTime[1] - this->totalTime[0],2) - 2*(this->pathPoses[1][i] - this->pathPoses[0][i])/this->acc[0][i]);
            }

            // Obtain timesb[n_changes-1][i]
            if (i == 5)
            {
                std::cout << " The terms for timesb[n-1][5]: " << (this->pathPoses[n-1][i] - this->pathPoses[n-2][i]) << ", " << pow(this->totalTime[n-1] - this->totalTime[n-2],2) - 2*abs((this->pathPoses[n-1][i] - this->pathPoses[n-2][i])/this->acc[n-1][i]) << std::endl;
            }
            if (this->acc[n-1][i] == 0)
            {
                this->timesb[n-1][i] = 0.0;
            }
            else
            {
                this->timesb[n-1][i] = (this->totalTime[n-1] - this->totalTime[n-2]) - sqrt(pow(this->totalTime[n-1] - this->totalTime[n-2],2) - 2*abs((this->pathPoses[n-1][i] - this->pathPoses[n-2][i])/this->acc[n-1][i]));
            }

            // Obtain speed[0][i]
            this->speed[0][i] = (this->pathPoses[1][i] - this->pathPoses[0][i])/(this->totalTime[1] - this->totalTime[0] - 0.5 * this->timesb[0][i]);

            // Obtain speed[n_slopes-1][i]
            this->speed[n-2][i] = (this->pathPoses[n-1][i] - this->pathPoses[n-2][i])/(this->totalTime[n-1] - this->totalTime[n-2] - 0.5 * this->timesb[n-1][i]);

            // Obtain acc[1][i]
            if ((this->speed[1][i] - this->speed[0][i]) > 0)
            {
                this->acc[1][i] = abs(this->blendAcceleration[1]);
            }
            else if ((this->speed[1][i] - this->speed[0][i]) < 0)
            {
                this->acc[1][i] = -abs(this->blendAcceleration[1]);
            }
            else
            {
                this->acc[1][i] = 0;
            }

            // Obtain acc[n-2][i]
            if ((this->speed[n-2][i] - this->speed[n-3][i]) > 0)
            {
                this->acc[n-2][i] = abs(this->blendAcceleration[n-2]);
            }
            else if ((this->speed[n-2][i] - this->speed[n-3][i]) < 0)
            {
                this->acc[n-2][i] = -abs(this->blendAcceleration[n-2]);
            }
            else
            {
                this->acc[n-2][i] = 0;
            }
        }

        std::cout << "The speeds in this state are: " << std::endl;
        for (uint i = 0 ; i < this->speed.size(); i++)
        {
            std::cout << "    " << i << ". " << this->speed[i] << std::endl;
        }

        std::cout << "The accelerations in this state are: " << std::endl;
        for (uint i = 0 ; i < this->acc.size(); i++)
        {
            std::cout << "    " << i << ". " << this->acc[i] << std::endl;
        }

        /////////////////////////////////////
        // Calculate timesb
        for (int i = 1; i < n - 1; i++) {
            for (int j = 0; j<6; j++){
                if (this->acc[i][j] == 0.0)
                {
                    this->timesb[i][j] = 0.0;    
                }
                else
                {
                    this->timesb[i][j] = abs((this->speed[i][j] - this->speed[i-1][j])/this->acc[i][j]); 
                }
            }
        }
        
        std::cout << "The values of timesb are:" << std::endl;
        for (int i = 0; i < n; i++)
        {
            std::cout << "    -"<<i<<". " << this->timesb[i] << std::endl;
        }

        /////////////////////////////////////
        // Calculate timesj

        for (int i = 0; i < 6; i++)
        {
            this->timesj[0][i] = (this->totalTime[1] - this->totalTime[0]) - this->timesb[0][i] - 0.5*timesb[1][i];
        }

        for (int i = 0; i < 6; i++)
        {
            this->timesj[n-2][i] = (this->totalTime[n-1] - this->totalTime[n-2])  - this->timesb[n-1][i] - 0.5*timesb[n-2][i];
        }

        for (int i = 1; i < n - 2; i++){
            for (int j = 0; j < 6; j++){
                this->timesj[i][j] = (this->totalTime[i+1] - this->totalTime[i]) - 0.5*timesb[i+1][j] - 0.5*timesb[i][j]; }}
        
        std::cout << "The values of timesj are:" << std::endl;
        for (int i = 0; i < n-1; i++)
        {
            std::cout << "    -"<<i<<". " << this->timesj[i] << std::endl;
        }


        /////////////////////////////////////
        // Calculate Accumulated times
        for (int i = 0; i < 6; i++)
        {
            this->accumulatedTimeb[0][i] = this->totalTime[0];
            this->accumulatedTimej[0][i] = this->accumulatedTimeb[0][i] + this->timesb[0][i];
        }

        for (int i = 1; i < n; i++)
        {
            this->accumulatedTimeb[i] = this->accumulatedTimej[i-1] + this->timesj[i-1];
            this->accumulatedTimej[i] = this->accumulatedTimeb[i]   + this->timesb[i];
        }

        std::cout << "The values of accumulatedTimej are:" << std::endl;
        for (int i = 0; i < n; i++)
        {
            std::cout << "    -"<<i<<". " << this->accumulatedTimej[i] << std::endl;
        }

        std::cout << "The values of accumulatedTimeb are:" << std::endl;
        for (int i = 0; i < n; i++)
        {
            std::cout << "    -"<<i<<". " << this->accumulatedTimeb[i] << std::endl;
        }

        /////////////////////////////////////
        // Calculate Initial joint speeds and positions
        for (int i = 0; i < 6; i++)
        {
            this->xInitialb[0][i] = this->pathPoses[0][i];
            this->xInitialj[0][i] = this->xInitialb[0][i] + 0.5*this->acc[0][i]*pow(this->timesb[0][i],2);
        }
        std::cout << "    [NOTE] xInitialb in position 0:  " << this->xInitialb[0] << std::endl;
        std::cout << "    [NOTE] xInitialj in position 0:  " << this->xInitialj[0] << std::endl;
        
        for (int i = 1; i < n-1; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                this->xInitialb[i][j] = this->xInitialj[i-1][j] + this->speed[i-1][j]*this->timesj[i-1][j];
                this->xInitialj[i][j] = this->xInitialb[i][j]   + this->speed[i-1][j]*this->timesb[i][j]    + 0.5*this->acc[i][j]*pow(timesb[i][j],2);
            }
            std::cout << "    [NOTE] xInitialb in position " << i << ":  " << this->xInitialb[i] << std::endl;
            std::cout << "    [NOTE] xInitialj in position " << i << ":  " << this->xInitialj[i] << std::endl;
        }
    }

};