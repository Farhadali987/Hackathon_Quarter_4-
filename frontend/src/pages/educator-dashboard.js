import React, { useState, useEffect } from 'react';
import styles from './EducatorDashboard.module.css';

const EducatorDashboard = () => {
  const [courses, setCourses] = useState([]);
  const [selectedCourse, setSelectedCourse] = useState(null);
  const [loading, setLoading] = useState(true);
  const [activeTab, setActiveTab] = useState('overview'); // overview, curriculum, students, analytics

  useEffect(() => {
    fetchCourses();
  }, []);

  const fetchCourses = async () => {
    try {
      setLoading(true);
      
      // In a real implementation, this would fetch from the backend
      // For this example, we'll create sample data
      const sampleCourses = [
        {
          id: 'course-1',
          title: 'Introduction to Physical AI',
          description: 'Basic concepts of Physical AI and Robotics',
          students: 24,
          progress: 65,
          lastAccessed: '2025-12-15'
        },
        {
          id: 'course-2',
          title: 'Humanoid Robotics Fundamentals',
          description: 'Understanding humanoid robot design and control',
          students: 18,
          progress: 42,
          lastAccessed: '2025-12-18'
        },
        {
          id: 'course-3',
          title: 'ROS 2 for Beginners',
          description: 'Getting started with Robot Operating System 2',
          students: 32,
          progress: 78,
          lastAccessed: '2025-12-20'
        }
      ];
      
      setCourses(sampleCourses);
      if (sampleCourses.length > 0) {
        setSelectedCourse(sampleCourses[0]);
      }
    } catch (error) {
      console.error('Error fetching courses:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleCourseSelect = (course) => {
    setSelectedCourse(course);
  };

  if (loading) {
    return (
      <div className={styles.dashboardContainer}>
        <div className={styles.loadingSpinner}>Loading dashboard...</div>
      </div>
    );
  }

  return (
    <div className={styles.dashboardContainer}>
      <header className={styles.dashboardHeader}>
        <h1>Educator Dashboard</h1>
        <p>Customize and manage your AI textbook content</p>
      </header>

      <div className={styles.dashboardLayout}>
        {/* Sidebar */}
        <aside className={styles.sidebar}>
          <nav className={styles.navMenu}>
            <button 
              className={`${styles.navButton} ${activeTab === 'overview' ? styles.active : ''}`}
              onClick={() => setActiveTab('overview')}
            >
              Overview
            </button>
            <button 
              className={`${styles.navButton} ${activeTab === 'curriculum' ? styles.active : ''}`}
              onClick={() => setActiveTab('curriculum')}
            >
              Curriculum
            </button>
            <button 
              className={`${styles.navButton} ${activeTab === 'students' ? styles.active : ''}`}
              onClick={() => setActiveTab('students')}
            >
              Students
            </button>
            <button 
              className={`${styles.navButton} ${activeTab === 'analytics' ? styles.active : ''}`}
              onClick={() => setActiveTab('analytics')}
            >
              Analytics
            </button>
          </nav>
        </aside>

        {/* Main Content */}
        <main className={styles.mainContent}>
          {activeTab === 'overview' && (
            <OverviewTab 
              courses={courses} 
              selectedCourse={selectedCourse} 
              onSelectCourse={handleCourseSelect}
            />
          )}
          {activeTab === 'curriculum' && (
            <CurriculumTab course={selectedCourse} />
          )}
          {activeTab === 'students' && (
            <StudentsTab course={selectedCourse} />
          )}
          {activeTab === 'analytics' && (
            <AnalyticsTab course={selectedCourse} />
          )}
        </main>
      </div>
    </div>
  );
};

// Overview Tab Component
const OverviewTab = ({ courses, selectedCourse, onSelectCourse }) => {
  return (
    <div className={styles.tabContent}>
      <h2>Course Overview</h2>
      
      <div className={styles.courseGrid}>
        {courses.map(course => (
          <div 
            key={course.id} 
            className={`${styles.courseCard} ${selectedCourse?.id === course.id ? styles.selected : ''}`}
            onClick={() => onSelectCourse(course)}
          >
            <h3>{course.title}</h3>
            <p>{course.description}</p>
            <div className={styles.courseStats}>
              <span className={styles.stat}>
                <strong>{course.students}</strong> Students
              </span>
              <span className={styles.stat}>
                <strong>{course.progress}%</strong> Progress
              </span>
            </div>
            <small>Last accessed: {course.lastAccessed}</small>
          </div>
        ))}
      </div>
      
      {selectedCourse && (
        <div className={styles.selectedCourseDetail}>
          <h3>Selected Course: {selectedCourse.title}</h3>
          <p>{selectedCourse.description}</p>
          
          <div className={styles.courseActions}>
            <button className={styles.actionButton}>Edit Content</button>
            <button className={styles.actionButton}>View Students</button>
            <button className={styles.actionButton}>Download Reports</button>
          </div>
        </div>
      )}
    </div>
  );
};

// Curriculum Tab Component
const CurriculumTab = ({ course }) => {
  const [chapters, setChapters] = useState([
    { id: 1, title: 'Introduction to Physical AI', status: 'published', order: 1 },
    { id: 2, title: 'ROS 2 Basics', status: 'published', order: 2 },
    { id: 3, title: 'Gazebo Simulation', status: 'published', order: 3 },
    { id: 4, title: 'NVIDIA Isaac Platform', status: 'draft', order: 4 },
    { id: 5, title: 'VLA Models', status: 'draft', order: 5 },
  ]);

  const [newChapter, setNewChapter] = useState({ title: '', order: 0 });

  const addChapter = () => {
    if (newChapter.title.trim()) {
      const newCh = {
        id: chapters.length + 1,
        title: newChapter.title,
        status: 'draft',
        order: newChapter.order || chapters.length + 1
      };
      setChapters([...chapters, newCh]);
      setNewChapter({ title: '', order: 0 });
    }
  };

  return (
    <div className={styles.tabContent}>
      <h2>Curriculum Management</h2>
      
      <div className={styles.chapterManagement}>
        <h3>Chapters for {course?.title}</h3>
        
        <div className={styles.newChapterForm}>
          <input
            type="text"
            placeholder="New chapter title"
            value={newChapter.title}
            onChange={(e) => setNewChapter({...newChapter, title: e.target.value})}
          />
          <input
            type="number"
            placeholder="Order"
            value={newChapter.order}
            onChange={(e) => setNewChapter({...newChapter, order: parseInt(e.target.value) || 0})}
          />
          <button onClick={addChapter}>Add Chapter</button>
        </div>
        
        <div className={styles.chaptersList}>
          {chapters
            .sort((a, b) => a.order - b.order)
            .map(chapter => (
              <div key={chapter.id} className={styles.chapterItem}>
                <span className={styles.chapterOrder}>{chapter.order}.</span>
                <span className={styles.chapterTitle}>{chapter.title}</span>
                <span className={`${styles.chapterStatus} ${styles[chapter.status]}`}>
                  {chapter.status}
                </span>
                <div className={styles.chapterActions}>
                  <button>Edit</button>
                  <button>Delete</button>
                </div>
              </div>
            ))}
        </div>
      </div>
    </div>
  );
};

// Students Tab Component
const StudentsTab = ({ course }) => {
  const [students, setStudents] = useState([
    { id: 1, name: 'Ali Khan', progress: 85, lastActive: '2025-12-18', status: 'active' },
    { id: 2, name: 'Sara Ahmed', progress: 92, lastActive: '2025-12-19', status: 'active' },
    { id: 3, name: 'Ahmed Raza', progress: 45, lastActive: '2025-12-15', status: 'inactive' },
    { id: 4, name: 'Fatima Ali', progress: 78, lastActive: '2025-12-20', status: 'active' },
    { id: 5, name: 'Omer Farooq', progress: 23, lastActive: '2025-12-10', status: 'inactive' },
  ]);

  return (
    <div className={styles.tabContent}>
      <h2>Student Management</h2>
      
      <div className={styles.studentManagement}>
        <h3>Students in {course?.title}</h3>
        
        <div className={styles.studentsTable}>
          <div className={styles.tableHeader}>
            <div>Name</div>
            <div>Progress</div>
            <div>Last Active</div>
            <div>Status</div>
            <div>Actions</div>
          </div>
          
          {students.map(student => (
            <div key={student.id} className={styles.tableRow}>
              <div>{student.name}</div>
              <div>
                <div className={styles.progressBar}>
                  <div 
                    className={styles.progressFill} 
                    style={{ width: `${student.progress}%` }}
                  ></div>
                </div>
                <span>{student.progress}%</span>
              </div>
              <div>{student.lastActive}</div>
              <div>
                <span className={`${styles.statusBadge} ${styles[student.status]}`}>
                  {student.status}
                </span>
              </div>
              <div>
                <button>View</button>
                <button>Message</button>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

// Analytics Tab Component
const AnalyticsTab = ({ course }) => {
  const [analyticsData, setAnalyticsData] = useState({
    totalStudents: 24,
    avgProgress: 65,
    completionRate: 42,
    activeStudents: 18,
    mostViewed: 'Chapter 2: ROS 2 Basics',
    leastViewed: 'Chapter 5: VLA Models'
  });

  return (
    <div className={styles.tabContent}>
      <h2>Course Analytics</h2>
      
      <div className={styles.analyticsDashboard}>
        <h3>Performance Metrics for {course?.title}</h3>
        
        <div className={styles.metricsGrid}>
          <div className={styles.metricCard}>
            <h4>Total Students</h4>
            <p className={styles.metricValue}>{analyticsData.totalStudents}</p>
          </div>
          <div className={styles.metricCard}>
            <h4>Avg. Progress</h4>
            <p className={styles.metricValue}>{analyticsData.avgProgress}%</p>
          </div>
          <div className={styles.metricCard}>
            <h4>Completion Rate</h4>
            <p className={styles.metricValue}>{analyticsData.completionRate}%</p>
          </div>
          <div className={styles.metricCard}>
            <h4>Active Students</h4>
            <p className={styles.metricValue}>{analyticsData.activeStudents}</p>
          </div>
        </div>
        
        <div className={styles.insightsSection}>
          <h4>Insights</h4>
          <ul>
            <li><strong>Most Viewed:</strong> {analyticsData.mostViewed}</li>
            <li><strong>Least Viewed:</strong> {analyticsData.leastViewed}</li>
            <li>Consider adding more engaging content to underperforming chapters</li>
            <li>Student engagement peaks in the middle of the course</li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default EducatorDashboard;