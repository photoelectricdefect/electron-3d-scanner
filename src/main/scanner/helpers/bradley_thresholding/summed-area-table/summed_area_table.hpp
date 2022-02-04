#ifndef SUMTABLE_H_
#define SUMTABLE_H_

#include <vector>

//TODO: Add r_a and r_b, i.e. the user should be able to specify rectangle side lengths

/**
 * @brief  Class made for convenient and efficient O(1) calculation of mean and standard deviation (in images etc.)
 * @note   User needs to be careful when specifiyng template types to avoid type conversion issues
 * @retval None
 */
template<typename TYPE_IN, typename TYPE_TABLE>
class summed_area_table
{
	private:
       /**
        * @brief  Builds the summed area table
        * @note   
        * @param  in: 2D data vector
        * @retval None
        */
        void build(const std::vector<std::vector<TYPE_IN>>& in) {
            int rows=in.size(),cols=in[0].size();
            table=std::vector<std::vector<TYPE_TABLE>>(rows,std::vector<TYPE_TABLE>(cols,0));

            for(size_t x=0;x<cols;x++) {
                TYPE_TABLE sum=static_cast<TYPE_TABLE>(in[0][x]);

                if(x>0) sum+=table[0][x-1];

                table[0][x]=sum;
        	}

        	for(size_t y=0;y<rows;y++) {
                TYPE_TABLE sum=static_cast<TYPE_TABLE>(in[y][0]);

                if(y>0) sum+=table[y-1][0];

                table[y][0]=sum;
        	}

        	for(size_t x=1;x<cols;x++) {
            	for(size_t y=1;y<rows;y++) {
                    TYPE_TABLE C =static_cast<TYPE_TABLE>(in[y][x]),Ix=table[y][x-1],Iy=table[y-1][x],Ixy=table[y-1][x-1];
                    TYPE_TABLE I=C+Ix+Iy-Ixy;
                    table[y][x]=I;
            	}
        	}
    	}

        /**
         * @brief  Calculates number of pixels (area) inside window
         * @note   
         * @param  x: x coordinate of center of rectangular area
         * @param  y: y coordinate of center of rectangular area
         * @param  r: radius of rectangular area (the diameter of the rectangle will be 1+2*r)
         * @retval 
         */
        int win_area(int x,int y,int r) {
            int rows=table.size(),cols=table[0].size();
            int xbr=x+r,ybr=y+r;
            int xtl=x-r-1,ytl=y-r-1;

            if(xbr>=cols) xbr=cols-1;
            if(ybr>=rows) ybr=rows-1;

            int wtlx=xtl>=-1?xtl:-1,wtly=ytl>=-1?ytl:-1,wbrx=xbr,wbry=ybr;
            return (wbrx-wtlx)*(wbry-wtly);
        }
    public:
        std::vector<std::vector<TYPE_TABLE>> table;

        summed_area_table(const std::vector<std::vector<TYPE_IN>>& in) {
            build(in);
        }

        /**
         * @brief  Get sum of rectangular area in table
         * @note   
         * @param  x: x coordinate of center of rectangular area
         * @param  y: y coordinate of center of rectangular area
         * @param  r: radius of rectangular area (the diameter of the rectangle will be 1+2*r)
         * @retval returns sum of rectangular area
         */
        TYPE_TABLE sum(int x,int y,int r) {
            int rows=table.size(),cols=table[0].size();
            int xbr=x+r,ybr=y+r;
            int xtl=x-r-1,ytl=y-r-1;
            int xtr=x+r,ytr=y-r-1;
            int xbl=x-r-1,ybl=y+r;

            if(xbr>=cols) xbr=cols-1;
            if(ybr>=rows) ybr=rows-1;

            TYPE_TABLE br=table[ybr][xbr];
            TYPE_TABLE winsum=br;

            if(xtl>=0&&ytl>=0) {
                TYPE_TABLE tl=table[ytl][xtl];
                winsum+=tl;
            }

            if(xbl>=0) {
                if(ybl>=rows) ybl=rows-1;

                TYPE_TABLE bl=table[ybl][xbl];
                winsum-=bl;
            }

            if(ytr>=0) {
                if(xtr>=cols) xtr=cols-1;

                TYPE_TABLE tr=table[ytr][xtr];
                winsum-=tr;
            }

            return winsum;
        }

       /**
         * @brief  Get mean of rectangular area in table
         * @note   
         * @param  x: x coordinate of center of rectangular area
         * @param  y: y coordinate of center of rectangular area
         * @param  r: radius of rectangular area (the diameter of the rectangle will be 1+2*r)
         * @retval returns mean of rectangular area
         */
        double mean(int x,int y,int r) {
            double winsum=static_cast<double>(sum(x,y,r));
            int area=win_area(x,y,r);

            return winsum/area;
        };

        /**
         * @brief  Get variance of rectangular area in table
         * @note   when window size is large (>80) you get strange results (the variance is always 0 etc.), SOLVE
         * @param  table_squared: table generated with same 2D data vector with squared elements
         * @param  x: x coordinate of center of rectangular area
         * @param  y: y coordinate of center of rectangular area
         * @param  r: radius of rectangular area (the diameter of the rectangle will be 1+2*r)
         * @retval returns variance of rectangular area
         */
        double variance(summed_area_table& table_squared,int x,int y,int r) {
            double S1=static_cast<double>(sum(x,y,r));
            double S2=static_cast<double>(table_squared.sum(x,y,r));
            int area=win_area(x,y,r);

            return (S2-pow(S1,2)/area)/area;
        };
};

#endif
