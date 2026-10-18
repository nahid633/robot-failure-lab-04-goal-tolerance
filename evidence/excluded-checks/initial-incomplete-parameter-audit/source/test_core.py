import unittest
from core import configuration,differences,endpoint_metrics,POSITION_LIMIT


class EndpointTests(unittest.TestCase):
    def poses(self,error):
        return [dict(stamp=i*.03,x=1-error,y=.5,yaw=0.) for i in range(120)]

    def test_one_leaf_and_no_mutation(self):
        base={'controller_server':{'ros__parameters':{'general_goal_checker':{'xy_goal_tolerance':.25,'yaw_goal_tolerance':.25}}},'monitor':True}
        a=configuration(base,'baseline');b=configuration(base,'relaxed')
        self.assertEqual(differences(a,b),[('controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance',.25,.75)])
        self.assertEqual(base,a)

    def test_success_can_fail_fixed_position_requirement(self):
        r=endpoint_metrics(self.poses(.65),self.poses(.62),0.,2.,'succeeded')
        self.assertFalse(r['settled_position_pass'])
        self.assertAlmostEqual(r['terminal_world_error_m'],.65)

    def test_failure_near_target_is_not_accepted(self):
        r=endpoint_metrics(self.poses(.1),self.poses(.1),0.,2.,'failed')
        self.assertFalse(r['settled_position_pass'])

    def test_missing_stale_terminal_and_sparse_tail_rejected(self):
        for data in [[],self.poses(.2)[:30],self.poses(.2)[::10]]:
            with self.assertRaises(ValueError):endpoint_metrics(data,data,0.,2.,'succeeded')

    def test_terminal_is_not_last_post_result_sample(self):
        data=self.poses(.2)
        for row in data:
            if row['stamp']>2:row['x']=.4
        r=endpoint_metrics(data,self.poses(.2),0.,2.,'succeeded')
        self.assertAlmostEqual(r['terminal_world_error_m'],.2)
        self.assertFalse(r['settled_position_pass'])

    def test_fixed_requirement_accepts_close_success(self):
        r=endpoint_metrics(self.poses(.2),self.poses(.21),0.,2.,'succeeded')
        self.assertTrue(r['settled_position_pass'])
        self.assertEqual(POSITION_LIMIT,.30)


if __name__=='__main__':unittest.main()
