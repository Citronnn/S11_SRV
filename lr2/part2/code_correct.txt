from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def changeLine(self, env, direction):
        count = 14
        angle = 25
        img, _, _, _ = env.step([1, direction * angle])
        for i in range(count):
            img, _, _, _ = env.step([1, 0])
            env.render()
        img, _, _, _ = env.step([1, -direction * angle])

    def isClearOtherRoad(self, env, direction):
        angle = 25
        img, _, _, _ = env.step([0, direction * angle])
        env.render()
        rotateContour = self.getContour(env, img)
        img, _, _, _ = env.step([0, -direction * angle])
        env.render()
        return False if rotateContour else True


    def getContour(self, env, img):
        img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(img, (0, 130, 170), (2, 250, 255))
        contour, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contour

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        
        condition = True
        line_changed = False
        while condition:
            img, reward, done, info = env.step([1, 0])
            contour = self.getContour(env, img)
            if line_changed and not contour and self.isClearOtherRoad(env, -1):
                self.changeLine(env, -1)
                line_changed = False
            if contour:
                _, _, w, h = cv2.boundingRect(contour[0])
                if h > 130 and not line_changed:
                    self.changeLine(env, 1)
                    line_changed = True
            env.render()
