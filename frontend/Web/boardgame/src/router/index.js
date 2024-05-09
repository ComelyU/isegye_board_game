import { createRouter, createWebHistory } from 'vue-router'
import MainPage from '@/components/MainPage.vue'
import PartnerShip from '@/components/PartnerShip.vue'
import BranchInfo from '@/components/BranchInfo.vue'
import BrandIntroduce from '@/components/BrandIntroduce.vue'
import SeatsInfo from '@/components/SeatsInfo.vue'

const router = createRouter({
  history: createWebHistory(),
  routes: [
    {
      path: '/',
      name: 'MainPage',
      component: MainPage
    },
    {
      path: '/brand',
      name: 'BrandIntroduce',
      component: BrandIntroduce
    },
    {
        path: '/partnership',
        name: 'PartnerShip',
        component: PartnerShip
    },
    {
        path: '/branchinfo',
        name: 'BranchInfo',
        component: BranchInfo
    },
    {
      path: '/branchinfo/:storename/:storeid',
      name: 'SeatsInfo',
      component: SeatsInfo,
      props: true
    },
  ]
})

export default router
