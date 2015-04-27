import roslib; roslib.load_manifest('ml_classifiers')
import rospy
import ml_classifiers.srv
import ml_classifiers.msg

#Wrapper for calls to ROS classifier service and management of classifier data
class ClassifierWrapper:

    def __init__(self):
        #Set up Classifier service handles
        print 'Waiting for Classifier services...'
        rospy.wait_for_service("/ml_classifiers/create_classifier")
        self.add_class_data = rospy.ServiceProxy(
            "/ml_classifiers/add_class_data",
            ml_classifiers.srv.AddClassData, persistent=True)
        self.classify_data = rospy.ServiceProxy(
            "/ml_classifiers/classify_data",
            ml_classifiers.srv.ClassifyData, persistent=True)
        self.clear_classifier = rospy.ServiceProxy(
            "/ml_classifiers/clear_classifier",
            ml_classifiers.srv.ClearClassifier, persistent=True)
        self.create_classifier = rospy.ServiceProxy(
            "/ml_classifiers/create_classifier",
            ml_classifiers.srv.CreateClassifier, persistent=True)
        self.load_classifier = rospy.ServiceProxy(
            "/ml_classifiers/load_classifier",
            ml_classifiers.srv.LoadClassifier, persistent=True)
        self.save_classifier = rospy.ServiceProxy(
            "/ml_classifiers/save_classifier",
            ml_classifiers.srv.SaveClassifier, persistent=True)
        self.train_classifier = rospy.ServiceProxy(
            "/ml_classifiers/train_classifier",
            ml_classifiers.srv.TrainClassifier, persistent=True)
        print 'OK\n'


    def addClassDataPoint(self, identifier, target_class, p):
        req = ml_classifiers.srv.AddClassDataRequest()
        req.identifier = identifier
        dp = ml_classifiers.msg.ClassDataPoint()
        dp.point = p
        dp.target_class = target_class
        req.data.append(dp)
        resp = self.add_class_data(req)


    def addClassDataPoints(self, identifier, target_classes, pts):
        req = ml_classifiers.srv.AddClassDataRequest()
        req.identifier = identifier
        for i in xrange(len(pts)):
            dp = ml_classifiers.msg.ClassDataPoint()
            dp.point = pts[i]
            dp.target_class = target_classes[i]
            req.data.append(dp)
        resp = self.add_class_data(req)


    def classifyPoint(self, identifier, p):
        req = ml_classifiers.srv.ClassifyDataRequest()
        req.identifier = identifier
        dp = ml_classifiers.msg.ClassDataPoint()
        dp.point = p
        req.data.append(dp)
        resp = self.classify_data(req)
        return resp.classifications[0]


    def classifyPoints(self, identifier, pts):
        req = ml_classifiers.srv.ClassifyDataRequest()
        req.identifier = identifier
        for p in pts:
            dp = ml_classifiers.msg.ClassDataPoint()
            dp.point = p
            req.data.append(dp)

        resp = self.classify_data(req)
        return resp.classifications


    def clearClassifier(self, identifier):
        req = ml_classifiers.srv.ClearClassifierRequest()
        req.identifier = identifier
        resp = self.clear_classifier(req)


    def createClassifier(self, identifier, class_type):
        req = ml_classifiers.srv.CreateClassifierRequest()
        req.identifier = identifier
        req.class_type = class_type
        resp = self.create_classifier(req)


    def loadClassifier(self, identifier, class_type, filename):
        req = ml_classifiers.srv.LoadClassifierRequest()
        req.identifier = identifier
        req.class_type = class_type
        req.filename = filename
        resp = self.load_classifier(req)


    def saveClassifier(self, identifier, filename):
        req = ml_classifiers.srv.SaveClassifierRequest()
        req.identifier = identifier
        req.filename = filename
        resp = self.save_classifier(req)


    def trainClassifier(self, identifier):
        req = ml_classifiers.srv.TrainClassifierRequest()
        req.identifier = identifier
        resp = self.train_classifier(req)


if __name__ == '__main__':
    cw = ClassifierWrapper()
    cw.createClassifier('test','ml_classifiers/SVMClassifier')

    targs = ['1','1','2','2','3']
    pts = [[0.1,0.2],[0.3,0.1],[3.1,3.2],[3.3,4.1],[5.1,5.2]]
    cw.addClassDataPoints('test', targs, pts)
    cw.trainClassifier('test')

    testpts = [[0.0,0.0],[5.5,5.5],[2.9,3.6]]
    resp = cw.classifyPoints('test',testpts)
    print resp
