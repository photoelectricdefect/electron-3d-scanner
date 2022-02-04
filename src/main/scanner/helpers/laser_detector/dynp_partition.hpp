#ifndef DYNPPT_H_
#define DYNPPT_H_

#include <vector>
#include <limits>

/**
 * @brief  Fitness function for estimating partition fitness
 * @note   
 * @param  data: Data
 * @param  from: Partition start
 * @param  to: Partition end
 * @param  ptr: Pointer for passing extra data
 * @retval 
 */
template<typename T>
using fitness_function = double(*)(std::vector<T> data,int from,int to,void*ptr);

/**
 * @brief  Class for finding the optimal partitioning of an interval I, given a fitness function which calculates
 * the fitness of each window of size [min_winw,max_winw] on interval I, T(n)=O(N*(max_winw-min_winw+1))
 * @note   
 * @retval None
 */
template<typename T>
class dynp_partition {
  private:
  public:
  /**
   * @brief  
   * @note   
   * @param  data: Input data vector
   * @param  fn_fit: Fitness function
   * @param  min_winw: Minimum window size
   * @param  max_winw: Maximum window size
   * @param  ptr: Extra data passed as void pointer to be used in fitness function as user desires
   * @retval 
   */
    std::vector<int> partition(const std::vector<T>& data,const fitness_function<T>& fn_fit,size_t min_winw,size_t max_winw,void* ptr=nullptr) {
        std::vector<int> opt(data.size()+1,0);
        std::vector<int> costs(data.size()+1,0);

        for (int i = min_winw; i <= data.size(); i++)
        {
            double max_cost=-DBL_MAX;
            double opt_idx=0;

            for (int j=min_winw;j<=max_winw;j++)
            {
                int idx_p=i-j;

                if(idx_p<0) 
                    break;

                if(idx_p>0&&idx_p<min_winw)
                    continue;

                double cost=costs[idx_p]+fn_fit(data,idx_p,i-1,ptr);

                if(cost>max_cost) {
                    max_cost=cost;
                    opt_idx=idx_p;
                }
            }

            costs[i]=max_cost;
            opt[i]=opt_idx;
        }
        
        std::vector<int> partitions;
        int p=opt[data.size()];

        do
        {
            partitions.insert(partitions.begin(),p);
            p=opt[p];
        } while(p>0);
        
        return partitions;
    }
}; 

#endif