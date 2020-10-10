#ifndef LANE_MODEL_H
#define LANE_MODEL_H

#include "common.h"
#include "cluster.h"


namespace LaneModel{
    template<class PointDType>
    class LinkedNode {
    public:
        virtual ~LinkedNode() {}
        virtual LinkedNode* left() = 0;
        virtual LinkedNode* right() = 0;
        virtual void setLeft(LinkedNode<PointDType> *) = 0;
        virtual void setRight(LinkedNode<PointDType> *) = 0;
        virtual void setRightLeft(LinkedNode<PointDType> *right, LinkedNode<PointDType> *left) {
            setRight(right); setLeft(left);
        }
    };

    struct Mark {
        MarkType m_type;
        cv::Point2f m_position;
        Mark(MarkType type, cv::Point2f position) : m_type(type), m_position(position) {}
    };

    template <class PointDType> class Road;
    template <class PointDType> class Lane;


    template <class PointDType>
    class LinePoints {
        public:
            LinePoints() {
                m_refcnt = new unsigned(0);
                m_data = nullptr;
            }
            LinePoints(cluster::Cluster_<PointDType>& points) : LinePoints() {
                *this = points;
            }
            LinePoints(const LinePoints& other) : LinePoints() {
                *this = other;
            }
            template<class U> LinePoints(
                    cluster::Cluster_<U>& points) : LinePoints() {
                LinePoints<U> lp(points);
                *this = lp;
            }
            template<class U> LinePoints(
                    const LinePoints<U>& other) : LinePoints() {
                *this = other;
            }

            LinePoints& operator=(cluster::Cluster_<PointDType>& points){
                if(*m_refcnt > 1){
                    --*m_refcnt;
                    m_refcnt = new unsigned(1);
                    m_data = new cluster::Cluster_<PointDType>();
                }else if(*m_refcnt == 1){
                    m_data->clear();
                }else {
                    *m_refcnt = 1;
                    m_data = new cluster::Cluster_<PointDType>();
                }
                std::copy(std::make_move_iterator(begin(points)),
                          std::make_move_iterator(end  (points)),
                          std::back_inserter(*(m_data)));
                return *this;
            }

            LinePoints& operator=(const LinePoints& other){
                if(*m_refcnt > 1){
                    --*m_refcnt;
                    m_refcnt = other.m_refcnt;
                    ++*m_refcnt;
                }else if(*m_refcnt == 1){
                    delete m_refcnt;
                    delete m_data;
                    m_refcnt = other.m_refcnt;
                    ++*m_refcnt;
                }else {
                    delete m_refcnt;
                    m_refcnt = other.m_refcnt;
                    ++*m_refcnt;
                }
                m_data = other.m_data;
                return *this;
            }

            template<class U> LinePoints& operator=(
                    const cluster::Cluster_<U>& points){
                if(*m_refcnt > 1){
                    --*m_refcnt;
                    m_refcnt = new unsigned(1);
                    m_data = new cluster::Cluster_<PointDType>();
                }else if(*m_refcnt == 1){
                    m_data->clear();
                }else {
                    *m_refcnt = 1;
                    m_data = new cluster::Cluster_<PointDType>();
                }
                m_data->reserve(points.size());
                for(const cv::Point_<U>& src : points){
                    m_data->push_back(cv::Point_<PointDType>(
                                                  static_cast<PointDType>(src.x),
                                                  static_cast<PointDType>(src.y) ));
                }
                return *this;
            }

            template<class U> LinePoints& operator=(
                    const LinePoints<U>& other){
                *this = *other.data();
                return *this;
            }

            ~LinePoints(){
                if(*m_refcnt > 1){
                    --*m_refcnt;
                }else if(*m_refcnt == 1){
                    delete m_refcnt;
                    if (m_data) {
                        m_data->clear();
                        delete m_data;
                    }
                }else {
                    delete m_refcnt;
                }
            }

            cluster::Cluster_<PointDType> * data() const {return m_data;}

            friend inline std::ostream& operator<< (std::ostream & os, const LinePoints<PointDType> & self){
                if (*self.m_refcnt == 0) return os;
                for(auto p = self.data()->begin(); p != self.data()->end(); ++p){
                    os << '(' << p->x << ',' << p->y << ')';
                }
                return os;
            }
    private:
            unsigned * m_refcnt;
            cluster::Cluster_<PointDType> * m_data;
        };

    template <class PointDType>
    class Line : public LinkedNode<PointDType>{

    public:
        template<class U=PointDType>
        Line(cluster::Cluster_<U>&& points = cluster::Cluster_<PointDType>(),
             Lane<PointDType> * left_lane = nullptr,
             Lane<PointDType> * right_lane = nullptr
             ):m_points(points), m_left_lane(left_lane), m_right_lane(right_lane), linetype(TYPE_SOLID){
        }
        virtual ~Line(){}
        virtual LinkedNode<PointDType> * left() override {return m_left_lane;}
        virtual LinkedNode<PointDType> * right() override {return m_right_lane;}
        virtual void setLeft(LinkedNode<PointDType>* left_lane) override {
            m_left_lane = dynamic_cast<Lane<PointDType>* >(left_lane);
        }
        virtual void setRight(LinkedNode<PointDType>* right_lane) override {
            m_right_lane = dynamic_cast<Lane<PointDType>* >(right_lane);
        }

        friend inline std::ostream& operator<< (std::ostream & os, const Line<PointDType> & self){
            os << '[' << self.m_points << ']';
            return os;
        }

        bool operator<(Line<PointDType> other) const {
            return this < &other;
        }

        cluster::Cluster_<PointDType>& points() {return *m_points.data();}
        void change_points(cluster::Cluster_<PointDType>& points){m_points = points;}
        
        float distance;
        LineType linetype;


    private:
        friend class Road<PointDType>;
        LinePoints<PointDType> m_points;
        Lane<PointDType> * m_left_lane, * m_right_lane;
    };


    template <class PointDType>
    class Lane : public LinkedNode<PointDType>{
    public:
        float lanewidth;
        bool opposite_direction = false;
        cv::Point2f stop_point;
        std::vector<Mark> m_marks;
        virtual LinkedNode<PointDType> * left() override {return m_left_line;}
        virtual LinkedNode<PointDType> * right() override {return m_right_line;}
        virtual ~Lane(){}
        void setLeft(LinkedNode<PointDType> *left_line) override {
            m_left_line = dynamic_cast<Line<PointDType>*>(left_line);
        }
        void setRight(LinkedNode<PointDType> *right_line) override {
            m_right_line = dynamic_cast<Line<PointDType>*>(right_line);
        }
    private:
        Line<PointDType> *m_left_line;
        Line<PointDType> *m_right_line;
    };

    template <class PointDType>
    class Road {
    public:
        template <class U> Road(
                std::vector<cluster::Cluster_<U> > clusters
                ){
            {// init
                m_head = new Lane<PointDType>();
                m_tail = new Line<PointDType>();
                m_head->setRightLeft(nullptr, m_tail);
                m_tail->setRightLeft(m_head, nullptr);
                m_cur = m_head;
                m_lane_size = 0;
                if(clusters.size() == 0) return;
                std::sort(clusters.begin(), clusters.end(), cluster::less_than<U>);
            }
            {// fill data
                for (auto cluster = clusters.begin();
                     cluster != clusters.end(); ++cluster, ++m_lane_size) {
                    Line<PointDType> * new_line = new Line<PointDType>(static_cast<cluster::Cluster_<U> >(*cluster));
                    Lane<PointDType> * new_lane = new Lane<PointDType>();
                    m_cur->setLeft(new_line);
                    new_line->setRightLeft(m_cur, new_lane);
                    new_lane->setRightLeft(new_line, m_tail);
                    m_tail->setRight(new_lane);
                    m_cur = new_lane;
                }
            }
        } // end of Road()

        ~Road(){
            for(LinkedNode<PointDType> * item = m_head->left(); item != m_tail; item = item->left()){
                delete item->right();
            }
            delete m_tail;
        }

        friend inline std::ostream& operator<< (std::ostream & os, const Road<PointDType> & self){
            int idx = 0;
            for(LinkedNode<PointDType> * item = self.m_head->left(); item != self.m_tail; item = item->left()->left()){
                auto line = dynamic_cast<Line<PointDType>* >(item);
                os << std::endl << '[' << idx++ << "]: " << *line << std::endl;
            }
            return os;
        }

        Line<PointDType> *line(int idx) const {
            LinkedNode<PointDType> * item = m_head->left();
            for(; idx > 0 && item != m_tail; --idx, item = item->left()->left());
            return dynamic_cast<Line<PointDType>* >(item);
        }

        std::vector<Line<PointDType>*> lines() const {
            std::vector<Line<PointDType>*> lines;
            for(LinkedNode<PointDType> * item = m_head->left(); item != m_tail; item = item->left()->left()){
                lines.push_back(dynamic_cast<Line<PointDType>* >(item));
            }
            return lines;
        }

        std::vector<Lane<PointDType>*> lanes() const {
            std::vector<Lane<PointDType>*> lanes;
            for(LinkedNode<PointDType> * item = m_head->left();  item != m_tail && item->left()->left() != m_tail;  item = item->left()->left()){
                lanes.push_back(dynamic_cast<Lane<PointDType>* >(item->left()));
            }
            return lanes;
        }
		
		void insert_rightline(Line<PointDType>* line, cluster::Cluster_<float> cluster, LineType linetype=TYPE_SOLID){
			Line<PointDType> * new_line = new Line<PointDType>(std::move(cluster));
			new_line->linetype = linetype;
			Lane<PointDType> * new_lane = new Lane<PointDType>();
			line->right()->setLeft(new_line);
			new_line->setRightLeft(line->right(), new_lane);
			line->setRight(new_lane);
			new_lane->setRightLeft(new_line, line);
			m_lane_size ++;
		}
		
		void push_right(Line<PointDType>* line, float avg_lanewidth, LineType linetype=TYPE_SOLID){
		    std::vector<cv::Point_<float>> cluster;
			for (auto p = line->points().begin(); p != line->points().end(); ++p){
			    cluster.push_back(cv::Point_<float>(p->x + avg_lanewidth, p->y));
		    }
		    insert_rightline(line, cluster::Cluster_<float>(cluster), linetype);
		    dynamic_cast<Lane<float>*>(line->right())->lanewidth = avg_lanewidth;
		}
		
		void push_left(Line<PointDType>* line, float avg_lanewidth, LineType linetype=TYPE_SOLID){
		    std::vector<cv::Point_<float>> cluster;
			for (auto p = line->points().begin(); p != line->points().end(); ++p){
			    cluster.push_back(cv::Point_<float>(p->x - avg_lanewidth, p->y));
		    }
		    insert_rightline(dynamic_cast<Line<float>*>(line->left()->left()), cluster::Cluster_<float>(cluster), linetype);
		    dynamic_cast<Lane<float>*>(line->left())->lanewidth = avg_lanewidth;
		}
		
		void del_line(Line<PointDType>* line){
			line->right()->setLeft(line->left()->left());
			line->left()->left()->setRight(line->right());
            delete line->left();
            delete line;
		}

        void clear(){
            for(auto line : lines()){
                del_line(line);
            }
        }

        int cal_current_lane_id(float center_x){
            current_lane_id = -1;
            auto lines = this->lines();
            if (lines.size()>1) {
                ++current_lane_id;
                for (auto it_line = lines.begin() + 1; it_line != lines.end() - 1; ++it_line) {
                    auto line = *it_line;
                    if(line->points().front().x > center_x && !dynamic_cast<Lane<float>*>(line->left())->opposite_direction) current_lane_id++;
                }
            }
            return current_lane_id;
        }

        int get_current_lane_id(){
            return current_lane_id;
        }


    private:
        // arrange from right to left
        LinkedNode<PointDType> * m_head, * m_tail, * m_cur;
        size_t m_lane_size;
        int current_lane_id = -1;
    };

}

#endif // LANE_MODEL_H
