-- MySQL dump 10.13  Distrib 8.3.0, for macos14.2 (arm64)
--
-- Host: k10a706.p.ssafy.io    Database: isegye
-- ------------------------------------------------------
-- Server version	8.3.0

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `code_group`
--

DROP TABLE IF EXISTS `code_group`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `code_group` (
  `group_name` varchar(32) COLLATE utf8mb3_unicode_ci NOT NULL,
  `group_description` varchar(128) COLLATE utf8mb3_unicode_ci NOT NULL,
  PRIMARY KEY (`group_name`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `code_group`
--

LOCK TABLES `code_group` WRITE;
/*!40000 ALTER TABLE `code_group` DISABLE KEYS */;
INSERT INTO `code_group` (`group_name`, `group_description`) VALUES ('code group name sample','code group description sample'),('gameCateogry','게임 카테고리'),('gameTag','게임 태그');
/*!40000 ALTER TABLE `code_group` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `code_item`
--

DROP TABLE IF EXISTS `code_item`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `code_item` (
  `id` int NOT NULL AUTO_INCREMENT,
  `group_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `item_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `item_description` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  PRIMARY KEY (`id`),
  KEY `group_name` (`group_name`),
  CONSTRAINT `code_item_ibfk_1` FOREIGN KEY (`group_name`) REFERENCES `code_group` (`group_name`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=69 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `code_item`
--

LOCK TABLES `code_item` WRITE;
/*!40000 ALTER TABLE `code_item` DISABLE KEYS */;
INSERT INTO `code_item` (`id`, `group_name`, `item_name`, `item_description`) VALUES (1,'code group name sample','code item name sample','code item description sample'),(2,'gameTag','tag-00','0번 게임 태그'),(4,'gameTag','tag-01','1번 게임 태그'),(5,'gameCateogry','Abstract Strategy','추상적인 전략'),(6,'gameCateogry','Adventure','모험'),(7,'gameCateogry','American West','서부'),(8,'gameCateogry','Bluffing','블러핑'),(9,'gameCateogry','Card Game','카드 게임'),(10,'gameCateogry','City Building','도시 건설'),(11,'gameCateogry','Deduction','추론'),(12,'gameCateogry','Economic','경제'),(13,'gameCateogry','Exploration','탐구'),(14,'gameCateogry','Fantasy','판타지'),(15,'gameCateogry','Fighting','전투'),(16,'gameCateogry','Humor','유머'),(17,'gameCateogry','Medieval','중세'),(18,'gameCateogry','Memory','기억'),(19,'gameCateogry','Movies / Tv / Radio theme','영상 테마'),(20,'gameCateogry','Murder / Mystery','살인 / 미스테리'),(21,'gameCateogry','Negotiation','협상'),(22,'gameCateogry','Number','숫자'),(23,'gameCateogry','Party Game','파티 게임'),(24,'gameCateogry','Puzzle','퍼즐'),(25,'gameCateogry','Real-time','실시간'),(26,'gameCateogry','Renaissance','르네상스'),(27,'gameCateogry','Science Fiction','공상과학'),(28,'gameCateogry','Space Exploration','우주 탐험'),(29,'gameCateogry','Spies/Secret Agents','비밀 요원'),(30,'gameCateogry','Territory Building','영토 건설'),(31,'gameCateogry','Trivia','트리비아'),(32,'gameTag','감성','감성'),(33,'gameTag','개척','개척'),(34,'gameTag','공상과학','공상과학'),(35,'gameTag','공포','공포'),(36,'gameTag','기만','기만'),(37,'gameTag','노잼','노잼'),(38,'gameTag','두뇌','두뇌'),(39,'gameTag','디자인','디자인'),(40,'gameTag','땅따먹기','땅따먹기'),(41,'gameTag','레이스','레이스'),(42,'gameTag','마피아게임','마피아게임'),(43,'gameTag','모험','모험'),(44,'gameTag','미스테리','미스테리'),(45,'gameTag','바다','바다'),(46,'gameTag','보안관','보안관'),(47,'gameTag','블러핑','블러핑'),(48,'gameTag','삼파전','삼파전'),(49,'gameTag','상상','상상'),(50,'gameTag','서부','서부'),(51,'gameTag','서부시대','서부시대'),(52,'gameTag','수집','수집'),(53,'gameTag','순발력','순발력'),(54,'gameTag','시뮬레이션','시뮬레이션'),(55,'gameTag','실시간','실시간'),(56,'gameTag','우주','우주'),(57,'gameTag','전략','전략'),(58,'gameTag','정글','정글'),(59,'gameTag','주사위','주사위'),(60,'gameTag','중세','중세'),(61,'gameTag','카드','카드'),(62,'gameTag','카드게임','카드게임'),(63,'gameTag','타일','타일'),(64,'gameTag','탐험','탐험'),(65,'gameTag','파티','파티'),(66,'gameTag','파티게임','파티게임'),(67,'gameTag','퍼즐','퍼즐'),(68,'gameTag','호러','호러');
/*!40000 ALTER TABLE `code_item` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `customer`
--

DROP TABLE IF EXISTS `customer`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `customer` (
  `id` int NOT NULL AUTO_INCREMENT,
  `room_id` int DEFAULT NULL,
  `is_theme` int NOT NULL,
  `people_num` int NOT NULL,
  `start_time` datetime(6) DEFAULT NULL,
  `end_time` datetime(6) DEFAULT NULL,
  `room_fee` int NOT NULL DEFAULT '0',
  PRIMARY KEY (`id`),
  KEY `FK9q5moa3m32falsvehbvmb8ons` (`room_id`),
  CONSTRAINT `FK9q5moa3m32falsvehbvmb8ons` FOREIGN KEY (`room_id`) REFERENCES `room` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=78 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `customer`
--

LOCK TABLES `customer` WRITE;
/*!40000 ALTER TABLE `customer` DISABLE KEYS */;
INSERT INTO `customer` (`id`, `room_id`, `is_theme`, `people_num`, `start_time`, `end_time`, `room_fee`) VALUES (1,1,1,4,'2024-04-25 16:01:09.000000','2024-05-07 14:35:44.154142',1144),(2,2,1,4,'2024-04-25 16:04:11.000000','2024-05-07 14:36:08.424373',1144),(3,1,1,4,NULL,NULL,0),(4,1,1,2,'2024-05-02 16:58:53.343702','2024-05-07 14:36:18.650790',234),(5,1,1,2,'2024-05-07 14:22:15.543783','2024-05-07 14:36:21.877499',0),(6,1,1,2,'2024-05-07 14:35:58.518658','2024-05-07 14:45:20.410690',0),(7,1,1,2,'2024-05-07 14:45:11.520074','2024-05-07 14:47:41.362744',0),(8,1,1,2,'2024-05-07 14:47:36.147126','2024-05-07 14:54:27.883388',0),(9,1,1,2,'2024-05-07 14:54:21.077913','2024-05-07 15:07:06.609480',0),(10,1,1,1,'2024-05-07 15:01:54.674221','2024-05-07 15:07:13.871189',0),(11,1,1,2,'2024-05-07 15:07:01.277129','2024-05-07 15:17:34.621211',0),(12,1,1,2,'2024-05-07 15:16:57.973043','2024-05-07 16:18:56.709263',2),(13,1,1,2,'2024-05-07 15:23:50.730656','2024-05-07 16:19:00.750653',0),(14,1,1,2,'2024-05-07 16:19:39.893605','2024-05-07 16:31:29.912926',0),(15,1,1,2,'2024-05-07 16:21:14.309705','2024-05-07 16:31:32.909307',0),(16,1,1,2,'2024-05-07 16:31:26.655461','2024-05-07 16:39:10.751364',0),(17,1,1,2,'2024-05-07 16:39:02.251583','2024-05-07 17:02:45.098190',0),(18,1,1,2,'2024-05-07 16:59:23.218645','2024-05-07 17:02:47.410903',0),(19,1,1,2,'2024-05-07 17:02:34.926982','2024-05-07 17:04:55.318670',0),(20,1,1,2,'2024-05-07 17:04:59.129561','2024-05-07 17:07:41.282167',0),(21,1,1,2,'2024-05-07 17:05:52.963651','2024-05-07 17:07:43.853618',0),(22,1,1,2,'2024-05-07 17:07:47.443452','2024-05-07 17:08:29.879929',0),(23,1,1,2,'2024-05-07 17:08:36.789337','2024-05-07 17:08:43.748898',0),(24,1,1,3,'2024-05-07 17:17:46.443452','2024-05-07 17:44:59.904852',0),(25,1,1,2,'2024-05-07 17:42:35.161955','2024-05-07 17:45:02.375111',0),(26,1,1,2,'2024-05-07 17:48:41.185613','2024-05-07 17:49:44.633985',0),(27,1,1,2,'2024-05-07 17:50:55.533563','2024-05-07 17:55:41.660799',0),(28,1,1,2,'2024-05-07 17:55:44.231426','2024-05-07 17:57:38.644198',0),(29,1,1,2,'2024-05-07 17:57:39.900302','2024-05-07 18:00:19.697502',0),(30,1,1,2,'2024-05-08 10:30:11.153070','2024-05-08 12:18:21.759547',2),(31,1,1,2,'2024-05-08 12:18:23.683218','2024-05-08 12:21:09.766738',0),(32,1,1,2,'2024-05-08 12:21:18.180878',NULL,0),(33,1,1,2,'2024-05-08 12:26:58.007608','2024-05-08 12:29:29.649214',0),(34,1,1,2,'2024-05-08 12:29:35.867640','2024-05-08 12:29:44.005065',0),(35,1,1,2,'2024-05-08 12:29:52.809504','2024-05-08 12:30:17.924301',0),(36,1,1,2,'2024-05-08 12:40:30.319791','2024-05-08 12:49:43.024265',0),(37,1,1,2,'2024-05-08 12:49:56.757085',NULL,0),(38,1,1,2,'2024-05-08 13:12:16.275011',NULL,0),(39,1,1,2,'2024-05-09 14:58:42.084136',NULL,0),(40,1,1,2,'2024-05-09 15:03:44.821418','2024-05-09 15:04:01.001656',0),(41,1,1,2,'2024-05-09 15:04:08.373767','2024-05-09 15:17:54.336908',0),(42,1,1,2,'2024-05-09 15:18:02.463347','2024-05-09 15:18:34.146350',0),(43,1,1,2,'2024-05-09 15:18:40.220171','2024-05-09 15:19:10.330982',0),(44,1,1,2,'2024-05-09 15:19:17.580450',NULL,0),(45,1,1,2,'2024-05-09 15:42:17.275703','2024-05-09 15:42:54.604081',0),(46,1,1,2,'2024-05-09 15:43:01.795404','2024-05-13 11:05:17.309033',182),(47,1,1,2,'2024-05-13 09:33:52.717971','2024-05-13 09:43:15.332081',0),(48,1,1,2,'2024-05-13 10:21:28.993148','2024-05-13 11:03:59.888569',0),(49,1,1,3,'2024-05-13 11:04:21.310715','2024-05-13 11:10:56.052859',0),(50,1,1,2,'2024-05-13 11:11:29.098708','2024-05-14 13:46:43.365552',52),(51,1,1,2,'2024-05-13 11:11:45.166561',NULL,0),(52,1,1,2,'2024-05-13 14:31:48.210753','2024-05-13 14:31:50.339964',0),(53,1,1,2,'2024-05-13 14:32:06.622875','2024-05-13 14:52:39.456025',0),(54,1,1,2,'2024-05-13 14:52:42.559560','2024-05-14 12:37:24.064162',42),(55,1,1,2,'2024-05-14 12:37:28.891486','2024-05-14 12:41:09.705080',0),(56,1,1,2,'2024-05-14 12:41:13.391582','2024-05-14 13:09:57.110879',0),(57,1,1,2,'2024-05-14 13:10:07.386636','2024-05-14 13:43:31.587549',0),(58,1,1,2,'2024-05-14 13:43:35.687303','2024-05-14 13:55:18.162187',0),(59,1,1,2,'2024-05-14 13:48:43.718460','2024-05-14 13:55:43.223647',0),(60,1,1,2,'2024-05-14 13:55:21.137131','2024-05-14 13:56:02.299940',0),(61,1,1,2,'2024-05-14 13:56:05.482914','2024-05-14 15:54:24.925647',2),(62,1,1,2,'2024-05-14 13:57:20.832412','2024-05-14 15:17:26.708554',2),(63,1,1,2,'2024-05-14 15:54:29.093185','2024-05-14 16:01:02.457667',0),(64,1,1,2,'2024-05-14 16:01:05.594545','2024-05-14 17:37:30.164514',2),(65,1,1,2,'2024-05-14 17:37:32.848818','2024-05-14 18:16:04.574968',0),(66,1,1,2,'2024-05-16 01:38:56.881351',NULL,0),(67,1,1,2,'2024-05-16 10:28:36.080802','2024-05-16 15:41:38.160163',10),(68,1,1,2,'2024-05-16 11:16:08.124991','2024-05-16 12:39:29.686199',2),(69,1,1,2,'2024-05-16 12:53:23.479146',NULL,0),(70,1,1,2,'2024-05-16 14:38:58.760643','2024-05-16 14:47:40.094721',0),(71,1,1,2,'2024-05-16 14:47:42.943463','2024-05-16 15:17:18.457961',0),(72,1,1,2,'2024-05-16 15:19:31.159996','2024-05-16 16:44:43.457804',2),(73,1,1,2,'2024-05-16 16:14:13.990176',NULL,0),(74,1,1,2,'2024-05-16 16:45:04.350606','2024-05-16 19:16:19.497226',4),(75,1,1,4,'2024-05-16 19:19:01.438410','2024-05-16 19:24:37.025217',0),(76,1,0,1,'2024-05-16 19:25:10.861780','2024-05-16 19:30:31.714053',0),(77,1,1,2,'2024-05-16 20:30:00.575275',NULL,0);
/*!40000 ALTER TABLE `customer` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `game`
--

DROP TABLE IF EXISTS `game`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `game` (
  `id` int NOT NULL AUTO_INCREMENT,
  `game_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `game_detail` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `min_player` int NOT NULL,
  `max_player` int NOT NULL,
  `min_playtime` int NOT NULL,
  `max_playtime` int NOT NULL,
  `game_difficulty` float NOT NULL,
  `game_img_url` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `theme_id` int DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKox6tdvw0mm5klagbu2jgq1w6j` (`theme_id`),
  CONSTRAINT `FKox6tdvw0mm5klagbu2jgq1w6j` FOREIGN KEY (`theme_id`) REFERENCES `theme` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=31 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `game`
--

LOCK TABLES `game` WRITE;
/*!40000 ALTER TABLE `game` DISABLE KEYS */;
INSERT INTO `game` (`id`, `game_name`, `game_detail`, `min_player`, `max_player`, `min_playtime`, `max_playtime`, `game_difficulty`, `game_img_url`, `theme_id`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,'sample','sample',1,2,0,1,1,NULL,1,NULL,'2024-05-16 16:08:51.539124','2024-05-16 16:08:51.538008'),(3,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/52b5fa59-f493-4942-8264-b71e73c72012_IMG_0723.JPG',2,'2024-05-04 01:19:35.361043','2024-05-16 16:09:00.518051','2024-05-16 16:09:00.517594'),(4,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/65863b2d-adbe-4b38-8a38-d61042ba8319_IMG_0723.JPG',2,'2024-05-04 01:21:39.168668','2024-05-04 01:21:39.168668',NULL),(5,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/7fa7b87c-e6b1-479d-92fb-bea44d25a8a5_IMG_0723.JPG',2,'2024-05-04 01:25:58.710450','2024-05-04 01:25:58.710450',NULL),(7,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/4836283b-b80b-4909-a47f-a6df06588593_IMG_0723.JPG',2,'2024-05-04 01:30:55.433974','2024-05-04 01:30:55.433974',NULL),(8,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/e367b082-1617-423e-9e2e-b3ccce42a331_IMG_0723.JPG',2,'2024-05-04 01:35:59.236138','2024-05-04 01:35:59.236138',NULL),(9,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/ac4e9aa1-7722-44f6-b7da-b1a885c03b02_IMG_0723.JPG',2,'2024-05-04 01:48:29.841185','2024-05-04 01:48:29.841185',NULL),(10,'gameName','gameDetail',2,4,10,30,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/gameName/354b9fa2-7b51-4737-a078-fd59556735f9_IMG_0723.JPG',2,'2024-05-04 02:09:19.384287','2024-05-04 16:08:15.493872','2024-05-04 16:08:15.474324'),(11,'카르카손','타일을 놓아 성을 짓고 길을 이어 점수를 얻으세요. 아름다운 중세 도시 카르카손의 풍경을 함께 완성하세요. 간단한 규칙이지만 할 때마다 새롭고 재미있습니다. 배우기 쉬우면서도 무한한 재미를 보장하는 명작 최고의 보드게임 중 하나로 손꼽히는 카르카손을 지금 만나 보세요.',2,5,30,45,1.9064,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%B9%B4%EB%A5%B4%EC%B9%B4/2522bd89-40c0-4a51-8cbc-e1d47b955ab7_%EC%B9%B4%EB%A5%B4%EC%B9%B4%EC%86%90.png',4,'2024-05-16 13:03:32.643547','2024-05-16 13:03:32.643547',NULL),(12,'카탄','찬란하게 발전하는 카탄 섬과 점차 다가오는 위협! 여러분의 기여로 이제 카탄섬은 발전에 발전을 거듭하여 찬란한 문명을 꽃피우게 되었습니다. 그러나 발전에는 그늘이 뒤따르는법 카탄 섬의 풍요로움이 바다 건너 야만족의 주목을 끌었습니다. 야만족 무리가 들이닥치기 전에 문명을 발전시키고 강력한 기사단을 양성하여 다가오는 습격에 대비하세요.',3,4,60,120,2.3139,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%B9%B4%ED%83%84/60a563fb-6b79-409d-a1c6-6f68c9d7098e_%EC%B9%B4%ED%83%84.jpg',4,'2024-05-16 13:11:35.487858','2024-05-16 13:11:35.487858',NULL),(13,'스플렌더','중세 최고의 보석상이 되어보세요! 개발 카드를 통해서 보석을 차근차근 모아 보세요. 반짝이는 보석들을 모으면 이에 반한 귀족들이 당신에게 협력할 것입니다. 가장 많은 재산을 모아 최고의 보석상이 되어보세요.',2,4,30,30,1.7951,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%8A%A4%ED%94%8C%EB%A0%8C%EB%8D%94/df4e8bd3-3360-4e36-a51c-9c7565651015_%EC%8A%A4%ED%94%8C%EB%A0%8C%EB%8D%94.jpg',5,'2024-05-16 13:15:11.158132','2024-05-16 13:15:11.158132',NULL),(14,'아줄','어떤 타일로 왕궁 벽면을 꾸미면 좋을까? 스페인의 알람브라 궁전에 방문한 포르투갈 국왕 마누엘 1세는 궁전 벽면을 화려하게 장식한 아름다운 타일들에 반해버렸습니다. 포르투갈의 타일 장인이 되어 에보라 왕궁을 화려하게 수놓아 주세요!',2,4,30,45,1.7639,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%95%84%EC%A4%84/f2020574-806f-40e4-a19a-96bd353d4618_%EC%95%84%EC%A4%84.jpg',6,'2024-05-16 13:17:50.302710','2024-05-16 13:17:50.302710',NULL),(15,'시타델','중세… 귀족… 난무하는 권모술수 음모와 배신이 쉴 새 없이 펼쳐진다! 시타델은 수많은 사람에게 사랑받아온 보드게임입니다. 때로는 협력하고 때로는 배신하며 중세 최고의 도시를 만들어보세요.',2,8,20,60,2.0524,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%8B%9C%ED%83%80%EB%8D%B8/72527cdf-915d-477c-a10a-ace0753b2957_%EC%8B%9C%ED%83%80%EB%8D%B8.jpg',5,'2024-05-16 13:20:50.913047','2024-05-16 13:20:50.913047',NULL),(16,'로스트 시티','숨겨진 고대 도시를 탐험하세요! 세계 곳곳에 숨은 신비한 미개척지를 찾아보세요. 얼음지대 열대우림 사막 용암 동굴 바닷속 등등 사람의 손길이 오랫동안 끊겼던 곳에 막대한 보물이 숨어있습니다. 여러분이 탐험로를 충분히 길게 잇는다면 많은 보물을 찾아낼 수 있을 것입니다.',2,2,30,30,1.4921,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%A1%9C%EC%8A%A4%ED%8A%B8%20%EC%8B%9C%ED%8B%B0/96a66962-b66c-4b1f-bad6-ff39f059d244_%EB%A1%9C%EC%8A%A4%ED%8A%B8%20%EC%8B%9C%ED%8B%B0.jpg',7,'2024-05-16 13:24:22.953623','2024-05-16 13:24:22.953623',NULL),(17,'레지스탕스 아발론','레지스탕스 아발론은 가장 완성된 마피아게임이란 평가를 받는 게임입니다. 이 게임속에선 마피아와 시민이 아닌 아서왕의 충성스러운 기사들과 모드레드의 악한 수하들이 브리튼의 미래를 놓고 싸웁니다. 여러분은 두 진영 중 하나에 속한채 정체를 숨기고 임무를 완수해야 합니다.',5,10,30,30,1.7612,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%A0%88%EC%A7%80%EC%8A%A4%ED%83%95%EC%8A%A4%20%EC%95%84%EB%B0%9C%EB%A1%A0/05e54b93-2b45-4dcb-a48e-13fac20db7e3_%EB%A0%88%EC%A7%80%EC%8A%A4%ED%83%95%EC%8A%A4%20%EC%95%84%EB%B0%9C%EB%A1%A0.jpg',5,'2024-05-16 13:31:35.684477','2024-05-16 13:31:35.684477',NULL),(18,'뱅','때는 서부 개척 시대 저마다 목적이 다른 총잡이들이 한 마을에 모였습니다. 마을의 평화를 지키려는 보안관 보안관을 도와 악당들을 물리치려는 부관 보안관을 쓰러뜨리고 한탕 하려는 무법자 방해되는 이를 모두 제거하고 새로운 보안관이 되려는 배신자. 보안관만 빼고는 누가 어떤 목적을 가졌는지 아무도 모릅니다. 이 난투극의 한복판에서 목적을 이루는 것은 누구일까요? 총알 한 발 맥주 한 잔에 울고 웃는 고독한 총잡이들의 승부!',4,7,20,40,1.6334,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%B1%85/1b7ea731-aefd-4175-9b8a-98104a2f7a3a_%EB%B1%85.jpg',4,'2024-05-16 13:35:17.908010','2024-05-16 13:35:17.908010',NULL),(19,'스페이스 크루','스페이스 크루에선 플레이어들에게 다양한 임무가 주어집니다. 플레이어들은 서로 경쟁하는 것이 아니라 힘을 합쳐 함께 임무를 완수해야하죠. 하지만 우주공간에서의 통신은 까다롭기 때문에 서로간의 의사소통에 제한이 있다는 문제가 있어 온전히 힘을 합치기 어렵습니다. 이런 악조건 속에서도 무사히 임무를 완수하여 다시 지구로 안전하게 돌아오기를 기원합니다.',2,5,20,20,1.9937,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%8A%A4%ED%8E%98%EC%9D%B4%EC%8A%A4%20%ED%81%AC%EB%A3%A8/8196266d-6f90-44a2-90ff-647b8959d0b5_%ED%81%AC%EB%A3%A8.jpg',8,'2024-05-16 13:56:05.019553','2024-05-16 13:56:05.019553',NULL),(20,'클루','6명의 손님이 미스터리한 초대장을 받은 후 보든 \'바디\' 블랙의 집인 튜더 저택에 도착합니다. 블랙은 각각의 손님이 자신을 돕지 않을 수 없는 완벽한 협박 카드를 가지고 있다는것을 밝힙니다. 블랙을 돕지 않는 다면 자신들의 비밀이 만천하에 공개될 것입니다. 잠시후 블랙이 자리를 뜨자 손님들은 저마다 생각을 하기 위해 흩어집니다. 그때 어디선가 비명이 울리고 손님들은 블랙이 살해된 것을 발견합니다. 당신은 이 미스터리를 해결해야 합니다.',2,6,45,45,1.6563,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%ED%81%B4%EB%A3%A8/ea709715-c532-459e-aaff-ccd83d14227d_%ED%81%B4%EB%A3%A8.png',9,'2024-05-16 14:00:52.498742','2024-05-16 14:00:52.498742',NULL),(21,'루미큐브','루미큐브는 2-4명이 함께 즐기는 보드게임입니다. 가지고 있는 패를 잘 조합하여 바닥에 내려놓아 보세요. 바닥에 있는 패를 이용해서도 도합할 수 있으니 머리를 쉴 새 없이 써야 합니다. 조합을 어떻게 할 것인지 계획하고 그대로 실행해 보세요. 하면할 수록 재미있어지는 게임 루미큐브입니다.',2,4,60,60,1.7395,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%A3%A8%EB%AF%B8%ED%81%90%EB%B8%8C/42640bc9-4d1d-4d5b-be0b-718bb53ead14_%EB%A3%A8%EB%AF%B8%ED%81%90%EB%B8%8C.jpg',6,'2024-05-16 14:03:34.782439','2024-05-16 14:03:34.782439',NULL),(22,'태양너머로','죽어가는 지구를 뒤로하고… 인류는 태양 너머로 나아갑니다. 인류의 미래를 짊어지고 나아갈 강력한 진영 중 하나를 선택하고 진영의 지도자가 되어 거주할 수 있는 행성계로 여러분을 따르는 이를 인도하세요. 우주의 새로운 지배자가 되려면 그에 걸맞은 실력을 갖추어야 할 것 입니다.',2,4,60,120,3.0843,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%ED%83%9C%EC%96%91%EB%84%88%EB%A8%B8%EB%A1%9C/31cfcc57-06be-4e8d-9c55-d630ce205241_%ED%83%9C%EC%96%91%EB%84%88%EB%A8%B8%EB%A1%9C.jpg',8,'2024-05-16 14:05:59.153294','2024-05-16 14:05:59.153294',NULL),(23,'다빈치 코드','다빈치 코드에 숨겨진 비밀을 밝혀라! 누가 먼저 비밀코드를 풀 것인가? 여러분의 비밀코드는 숨기고 상대방의 코드를 추리하세요. 하나씩 드러나는 상대방의 코드! 정보를 바탕으로 나머지코드를 모두 맞혀 보세요.',2,4,15,15,1.4862,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%8B%A4%EB%B9%88%EC%B9%98%20%EC%BD%94%EB%93%9C/b861d39f-ef7a-49b1-8f75-9527bbf5787a_%EB%8B%A4%EB%B9%88%EC%B9%98%20%EC%BD%94%EB%93%9C.jpg',6,'2024-05-16 14:09:53.568407','2024-05-16 14:09:53.568407',NULL),(24,'블로커스','규칙은 단 하나 자기 블록의 꼭지점끼리만 맞닿게 하라! 블록으로 상대를 차단하고 자신만의 공간을 만드세요!',2,5,30,40,1.7039,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%B8%94%EB%A1%9C%EC%BB%A4%EC%8A%A4/be079c78-8b10-4437-af0c-f53d7650af39_%EB%B8%94%EB%A1%9C%EC%BB%A4%EC%8A%A4.jpg',6,'2024-05-16 14:12:28.675940','2024-05-16 14:12:28.675940',NULL),(25,'딕싯','멋진 그림을 보고 이야기를 나누면서 서로의 생각과 마음을 알아맞히는 게임 딕싯이 특별한 만남으로 찾아왔습니다. 수십 년간 사람들에게 아름다운 이야기를 선사해 온 디즈니와 픽사를 이제 딕싯의 세계에서 만나보세요. 사랑스러운 캐릭터들과 함께했던 장면들을 떠올리며 여러분의 상상력과 창의력으로 그 추억을 재탄생시켜 주세요!',3,6,30,30,1.2128,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%94%95%EC%8B%AF/8d865eeb-4c00-4c55-902b-786a8adf180b_%EB%94%95%EC%8B%AF.jpg',6,'2024-05-16 14:16:53.479795','2024-05-16 14:16:53.479795',NULL),(26,'미스터리 하우스','3D 입체 저택모형과 스마트폰 애플리케이션을 이용해 수수께끼를 풀어나가는 실시간 협력형 방 탈출 게임입니다. 모험마다 완전히 변모하는 입체 저택을 구석구석 탐험하세요. 숨겨진 단서를 찾아 수수께끼를 풀면 새로운 방이 열리며 이야기의 전말이 드러납니다. 모든 비밀을 풀어낸 사람만이 저택을 빠져나갈 수 있습니다. 용기 있게 저택의 비밀을 파헤칠 준비가 되셨나요?',1,5,60,60,2.3,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%AF%B8%EC%8A%A4%ED%84%B0%EB%A6%AC%20%ED%95%98%EC%9A%B0%EC%8A%A4/482de80f-f7a8-4017-a4df-9ed9f709b13a_%EB%AF%B8%EC%8A%A4%ED%84%B0%EB%A6%AC%20%ED%95%98%EC%9A%B0%EC%8A%A4.jpg',9,'2024-05-16 14:19:38.816270','2024-05-16 14:19:38.816270',NULL),(27,'13 클루','추리 게임의 새로운 진화! 13개의 단서에서 사건의 진상을 파헤쳐라! 연쇄살인으로 긴장감이 가득한 19세기 영국. 여러분은 런던의 일류 탐정이 되어 살인 사건의 진상을 파헤쳐야 합니다. 자신이 담당한 사건의 진상을 파헤치고 최고의 탐정으로 거듭나십시오.',2,6,30,30,1.8889,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/13%20%ED%81%B4%EB%A3%A8/7ae89a65-03a7-41a9-a00d-cdf6834b204f_13%20%ED%81%B4%EB%A3%A8.png',9,'2024-05-16 14:21:43.744508','2024-05-16 14:21:43.744508',NULL),(28,'5초 준다!','카드를 펼치고 주제에 맞는 단어 3가지를 빠르게 대답하세요. 시간은 단 5초! 정확하게 5초에 맞춰진 특별한 구슬 타이머가 여러분을 재촉할 것입니다.',3,6,30,30,1.1148,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/5%EC%B4%88%20%EC%A4%80%EB%8B%A4%21/15161538-b276-42e2-ab3e-a0c301a3a3c9_5%EC%B4%88%20%EC%A4%80%EB%8B%A4%21.jpg',6,'2024-05-16 14:23:58.101254','2024-05-16 14:23:58.101254',NULL),(29,'다이브','외딴 섬 윈드바크에는 오래된 전통이 있습니다. 이곳에서는 매년 하짓날, 성년을 맞은 마을의 젊은이들이 신성한 돌을 찾아 깊은 바닷속으로 뛰어듭니다. 돌을 가장 먼저 찾아오는 이가 그 해의 영웅이 됩니다. 상어는 피하고, 바다거북과 가오리의 도움을 받아 누구보다 먼저 깊이 내려가세요. 하지만 출렁이는 물결 너머 저 앞 그림자는 과연 상어일까요? 바다거북일까요?',1,4,20,30,1.6923,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EB%8B%A4%EC%9D%B4%EB%B8%8C/7cff25e0-de8f-4833-801b-4bf14c7d8d43_%EB%8B%A4%EC%9D%B4%EB%B8%8C.png',10,'2024-05-16 14:26:12.061206','2024-05-16 14:26:12.061206',NULL),(30,'시에라 웨스트','1840년대 후반, 수천명의 개척자들이 부와 기회를 위해서 서부로 이주했습니다. 이 용감한 영혼의 소유자들은 훗날 캘리포니아의 금맥이 될 시에라 네바다 산악을 마차로 여행해 왔습니다. 시에라웨스트에서 여러분들은 준비되고 강인한 개척자들의 그룹을 이끌고, 이들로 하여금 단계마다 다양한 전략을 시도합니다.',1,4,40,60,3.087,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameImg/%EC%8B%9C%EC%97%90%EB%9D%BC%20%EC%9B%A8%EC%8A%A4%ED%8A%B8/679f3a55-0076-4430-8e87-2a4f6ed7a526_%EC%8B%9C%EC%97%90%EB%9D%BC%20%EC%9B%A8%EC%8A%A4%ED%8A%B8.jpg',4,'2024-05-16 14:28:17.969687','2024-05-16 14:28:17.969687',NULL);
/*!40000 ALTER TABLE `game` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `game_tag_category`
--

DROP TABLE IF EXISTS `game_tag_category`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `game_tag_category` (
  `id` int NOT NULL AUTO_INCREMENT,
  `game_id` int DEFAULT NULL,
  `group_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `code_item_id` int DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKt21xw6v8larc7f2fnrm5kou9u` (`game_id`),
  KEY `group_name` (`group_name`),
  KEY `code_item_id` (`code_item_id`),
  CONSTRAINT `FKt21xw6v8larc7f2fnrm5kou9u` FOREIGN KEY (`game_id`) REFERENCES `game` (`id`),
  CONSTRAINT `game_tag_category_ibfk_1` FOREIGN KEY (`group_name`) REFERENCES `code_group` (`group_name`),
  CONSTRAINT `game_tag_category_ibfk_2` FOREIGN KEY (`code_item_id`) REFERENCES `code_item` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=152 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `game_tag_category`
--

LOCK TABLES `game_tag_category` WRITE;
/*!40000 ALTER TABLE `game_tag_category` DISABLE KEYS */;
INSERT INTO `game_tag_category` (`id`, `game_id`, `group_name`, `code_item_id`) VALUES (1,1,'code group name sample',1),(4,3,'gameTag',2),(5,3,'gameTag',4),(6,4,'gameTag',2),(7,4,'gameTag',4),(8,5,'gameTag',2),(9,5,'gameTag',4),(12,7,'gameTag',2),(13,7,'gameTag',4),(14,8,'gameTag',2),(15,8,'gameTag',4),(16,9,'gameTag',2),(17,9,'gameTag',4),(18,10,'gameTag',2),(19,10,'gameTag',4),(20,11,'gameCateogry',10),(21,11,'gameCateogry',17),(22,11,'gameCateogry',30),(23,11,'gameTag',40),(24,11,'gameTag',60),(25,11,'gameTag',63),(26,11,'gameTag',57),(27,12,'gameTag',59),(28,12,'gameTag',63),(29,12,'gameTag',41),(30,12,'gameCateogry',12),(31,12,'gameCateogry',21),(32,13,'gameCateogry',9),(33,13,'gameCateogry',12),(34,13,'gameCateogry',26),(35,13,'gameTag',62),(36,13,'gameTag',41),(37,13,'gameTag',60),(38,14,'gameCateogry',5),(39,14,'gameCateogry',24),(40,14,'gameCateogry',26),(41,14,'gameTag',67),(42,14,'gameTag',39),(43,14,'gameTag',63),(44,15,'gameCateogry',8),(45,15,'gameCateogry',9),(46,15,'gameCateogry',10),(47,15,'gameCateogry',11),(48,15,'gameCateogry',14),(49,15,'gameCateogry',17),(50,15,'gameTag',60),(51,15,'gameTag',61),(52,15,'gameTag',36),(53,15,'gameTag',52),(54,16,'gameCateogry',9),(55,16,'gameCateogry',13),(56,16,'gameTag',64),(57,16,'gameTag',58),(58,16,'gameTag',43),(59,17,'gameCateogry',8),(60,17,'gameCateogry',9),(61,17,'gameCateogry',11),(62,17,'gameCateogry',14),(63,17,'gameCateogry',17),(64,17,'gameCateogry',21),(65,17,'gameCateogry',23),(66,17,'gameCateogry',29),(67,17,'gameTag',42),(68,17,'gameTag',66),(69,17,'gameTag',60),(70,18,'gameCateogry',7),(71,18,'gameCateogry',8),(72,18,'gameCateogry',9),(73,18,'gameCateogry',11),(74,18,'gameCateogry',15),(75,18,'gameTag',62),(76,18,'gameTag',48),(77,18,'gameTag',51),(78,18,'gameTag',46),(79,19,'gameCateogry',9),(80,19,'gameCateogry',27),(81,19,'gameCateogry',13),(82,19,'gameTag',62),(83,19,'gameTag',34),(84,19,'gameTag',57),(85,19,'gameTag',56),(86,20,'gameCateogry',8),(87,20,'gameCateogry',11),(88,20,'gameCateogry',19),(89,20,'gameCateogry',20),(90,20,'gameTag',35),(91,20,'gameTag',68),(92,20,'gameTag',44),(93,20,'gameTag',47),(94,21,'gameCateogry',5),(95,21,'gameCateogry',9),(96,21,'gameCateogry',22),(97,21,'gameTag',38),(98,21,'gameTag',67),(99,21,'gameTag',62),(100,22,'gameCateogry',27),(101,22,'gameCateogry',28),(102,22,'gameTag',34),(103,22,'gameTag',57),(104,22,'gameTag',56),(105,23,'gameCateogry',11),(106,23,'gameTag',38),(107,24,'gameCateogry',5),(108,24,'gameTag',38),(109,24,'gameTag',57),(110,24,'gameTag',67),(111,24,'gameTag',40),(112,25,'gameCateogry',9),(113,25,'gameCateogry',16),(114,25,'gameCateogry',23),(115,25,'gameTag',41),(116,25,'gameTag',54),(117,25,'gameTag',37),(118,25,'gameTag',32),(119,25,'gameTag',49),(120,26,'gameCateogry',6),(121,26,'gameCateogry',11),(122,26,'gameCateogry',20),(123,26,'gameCateogry',23),(124,26,'gameCateogry',25),(125,26,'gameTag',43),(126,26,'gameTag',44),(127,26,'gameTag',68),(128,27,'gameCateogry',11),(129,27,'gameCateogry',18),(130,27,'gameCateogry',20),(131,27,'gameTag',35),(132,27,'gameTag',68),(133,27,'gameTag',44),(134,28,'gameCateogry',9),(135,28,'gameCateogry',23),(136,28,'gameCateogry',25),(137,28,'gameCateogry',31),(138,28,'gameTag',55),(139,28,'gameTag',53),(140,28,'gameTag',65),(141,29,'gameCateogry',9),(142,29,'gameCateogry',23),(143,29,'gameCateogry',31),(144,29,'gameTag',61),(145,29,'gameTag',45),(146,29,'gameTag',64),(147,30,'gameCateogry',9),(148,30,'gameTag',61),(149,30,'gameTag',50),(150,30,'gameTag',33),(151,30,'gameTag',63);
/*!40000 ALTER TABLE `game_tag_category` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `menu`
--

DROP TABLE IF EXISTS `menu`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `menu` (
  `id` int NOT NULL AUTO_INCREMENT,
  `store_id` int DEFAULT NULL,
  `menu_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `menu_type` char(1) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `menu_price` int NOT NULL,
  `is_available` int NOT NULL,
  `menu_img_url` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FK4sgenfcmk1jajhgctnkpn5erg` (`store_id`),
  CONSTRAINT `FK4sgenfcmk1jajhgctnkpn5erg` FOREIGN KEY (`store_id`) REFERENCES `store` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=10 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `menu`
--

LOCK TABLES `menu` WRITE;
/*!40000 ALTER TABLE `menu` DISABLE KEYS */;
INSERT INTO `menu` (`id`, `store_id`, `menu_name`, `menu_type`, `menu_price`, `is_available`, `menu_img_url`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,1,'아메리카노','C',4500,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/coffee/%E1%84%8B%E1%85%A1%E1%84%86%E1%85%A6%E1%84%85%E1%85%B5%E1%84%8F%E1%85%A1%E1%84%82%E1%85%A9.png','2024-05-03 10:18:59.998685','2024-05-03 10:18:59.998685',NULL),(2,1,'콜라','D',2000,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/drink/%E1%84%8F%E1%85%A9%E1%86%AF%E1%84%85%E1%85%A1.png','2024-05-03 10:20:35.086762','2024-05-03 10:20:35.086762',NULL),(3,1,'감자튀김','F',3000,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/food/%E1%84%80%E1%85%A1%E1%86%B7%E1%84%8C%E1%85%A1%E1%84%90%E1%85%B1%E1%84%80%E1%85%B5%E1%86%B7M.png','2024-05-03 10:22:30.067083','2024-05-03 10:22:30.067083',NULL),(4,1,'크리스피 순살치킨','F',15000,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/food/%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%84%89%E1%85%B3%E1%84%91%E1%85%B5%E1%84%89%E1%85%AE%E1%86%AB%E1%84%89%E1%85%A1%E1%86%AF%E1%84%8E%E1%85%B5%E1%84%8F%E1%85%B5%E1%86%AB.png','2024-05-17 11:51:12.953224','2024-05-17 11:51:12.953224',NULL),(5,1,'소떡소떡','F',2500,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/food/%E1%84%89%E1%85%A9%E1%84%84%E1%85%A5%E1%86%A8%E1%84%89%E1%85%A9%E1%84%84%E1%85%A5%E1%86%A8%E1%84%92%E1%85%A9%E1%86%B7%E1%84%91%E1%85%A6%E1%84%8B%E1%85%B5%E1%84%8C%E1%85%B5.png','2024-05-17 11:52:19.040050','2024-05-17 11:52:19.040050',NULL),(6,1,'카페 라떼','C',5500,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/coffee/%E1%84%8F%E1%85%A1%E1%84%91%E1%85%A6%E1%84%85%E1%85%A1%E1%84%84%E1%85%A6.png','2024-05-17 11:53:16.155185','2024-05-17 11:53:16.155185',NULL),(7,1,'바닐라 라떼','C',5500,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/coffee/%E1%84%87%E1%85%A1%E1%84%82%E1%85%B5%E1%86%AF%E1%84%85%E1%85%A1%E1%84%85%E1%85%A1%E1%84%84%E1%85%A6.png','2024-05-17 11:53:37.390153','2024-05-17 11:53:37.390153',NULL),(8,1,'복숭아 아이스티','D',5000,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/drink/%E1%84%8B%E1%85%A1%E1%84%8B%E1%85%B5%E1%84%89%E1%85%B3%E1%84%90%E1%85%B5.png','2024-05-17 11:55:09.558732','2024-05-17 11:55:09.558732',NULL),(9,1,'체리콕','D',6000,1,'https://accio-isegye.s3.ap-northeast-2.amazonaws.com/menuImg/drink/%E1%84%8E%E1%85%A6%E1%84%85%E1%85%B5%E1%84%8F%E1%85%A9%E1%86%A8.png','2024-05-17 11:55:55.968629','2024-05-17 11:55:55.968629',NULL);
/*!40000 ALTER TABLE `menu` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `order_game`
--

DROP TABLE IF EXISTS `order_game`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `order_game` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `customer_id` int DEFAULT NULL,
  `stock_id` int DEFAULT NULL,
  `order_type` int NOT NULL,
  `order_status` int NOT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `delivered_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKsckwl4e01euj7n5va07d9csro` (`customer_id`),
  KEY `FK73tohaymnx4vgrfnpiswbafks` (`stock_id`),
  CONSTRAINT `FK73tohaymnx4vgrfnpiswbafks` FOREIGN KEY (`stock_id`) REFERENCES `stock` (`id`),
  CONSTRAINT `FKsckwl4e01euj7n5va07d9csro` FOREIGN KEY (`customer_id`) REFERENCES `customer` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=85 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `order_game`
--

LOCK TABLES `order_game` WRITE;
/*!40000 ALTER TABLE `order_game` DISABLE KEYS */;
INSERT INTO `order_game` (`id`, `customer_id`, `stock_id`, `order_type`, `order_status`, `created_at`, `delivered_at`) VALUES (1,3,2,0,2,'2024-05-06 07:36:42.284257','2024-05-06 07:39:13.187587'),(2,3,1,0,2,'2024-05-06 11:42:29.951318','2024-05-14 18:00:34.041321'),(3,3,1,0,2,'2024-05-06 11:46:49.196628','2024-05-10 17:09:42.074333'),(4,15,1,0,0,'2024-05-07 16:21:40.082935',NULL),(5,15,1,1,0,'2024-05-07 16:24:58.252204',NULL),(6,15,1,1,0,'2024-05-07 16:25:13.124728',NULL),(7,15,1,1,0,'2024-05-07 16:25:27.631737',NULL),(8,15,1,0,0,'2024-05-07 16:26:45.702327',NULL),(9,15,1,1,0,'2024-05-07 16:27:02.233262',NULL),(10,15,1,0,0,'2024-05-07 16:28:26.947864',NULL),(11,15,1,0,0,'2024-05-07 16:30:07.793157',NULL),(12,16,1,0,2,'2024-05-07 16:31:40.795099','2024-05-10 15:34:05.826838'),(13,16,1,1,2,'2024-05-07 16:31:45.576412','2024-05-10 15:42:51.039288'),(14,24,1,0,0,'2024-05-07 17:17:58.909195',NULL),(15,31,1,0,0,'2024-05-08 12:18:48.138953',NULL),(16,31,1,1,0,'2024-05-08 12:18:54.150774',NULL),(17,35,1,0,0,'2024-05-08 12:30:11.432191',NULL),(18,35,1,1,0,'2024-05-08 12:30:13.571065',NULL),(19,37,1,0,0,'2024-05-08 12:51:44.958334',NULL),(20,37,1,1,0,'2024-05-08 12:51:48.282945',NULL),(21,38,1,0,0,'2024-05-08 13:12:25.193297',NULL),(22,38,1,1,0,'2024-05-08 13:12:31.533324',NULL),(23,38,2,0,3,'2024-05-08 13:12:43.823119',NULL),(24,38,2,1,3,'2024-05-08 13:12:49.306876',NULL),(25,54,1,0,0,'2024-05-13 15:36:10.332699',NULL),(26,54,1,1,0,'2024-05-13 15:45:22.299802',NULL),(27,54,1,0,0,'2024-05-13 15:56:59.054272',NULL),(28,54,1,1,0,'2024-05-13 15:57:07.247336',NULL),(29,54,1,0,0,'2024-05-13 16:01:23.836072',NULL),(30,54,1,1,0,'2024-05-13 16:01:28.677322',NULL),(31,56,1,0,0,'2024-05-14 13:01:32.359609',NULL),(32,57,1,0,0,'2024-05-14 13:10:16.875920',NULL),(33,57,1,0,0,'2024-05-14 13:31:20.512674',NULL),(34,57,1,1,0,'2024-05-14 13:31:23.222760',NULL),(35,57,1,0,0,'2024-05-14 13:37:07.643719',NULL),(36,58,1,0,0,'2024-05-14 13:43:38.191346',NULL),(37,59,1,0,0,'2024-05-14 13:54:03.166594',NULL),(38,60,1,0,0,'2024-05-14 13:55:26.545278',NULL),(39,61,1,0,0,'2024-05-14 13:56:24.920383',NULL),(40,62,1,0,0,'2024-05-14 13:59:12.519968',NULL),(41,61,1,0,0,'2024-05-14 14:54:49.222683',NULL),(42,61,1,0,0,'2024-05-14 15:01:00.941544',NULL),(43,61,1,0,3,'2024-05-14 15:42:06.179715',NULL),(44,61,1,0,3,'2024-05-14 15:43:34.450134',NULL),(45,61,1,0,0,'2024-05-14 15:46:05.926031',NULL),(46,61,1,0,2,'2024-05-14 15:46:43.192409','2024-05-14 15:47:24.090866'),(47,61,1,1,2,'2024-05-14 15:47:40.414629','2024-05-14 15:47:53.205676'),(48,61,1,0,2,'2024-05-14 15:48:17.163863','2024-05-14 15:48:25.939150'),(49,61,1,1,2,'2024-05-14 15:48:37.548614','2024-05-14 15:53:51.161580'),(50,64,2,0,2,'2024-05-14 16:03:20.278176','2024-05-14 16:03:48.865094'),(51,64,2,1,3,'2024-05-14 16:04:00.932296',NULL),(52,64,1,0,3,'2024-05-14 16:04:00.932343',NULL),(53,64,1,1,2,'2024-05-14 16:04:21.508683','2024-05-14 16:09:08.717388'),(54,64,1,0,3,'2024-05-14 16:09:35.143194',NULL),(55,64,1,0,3,'2024-05-14 16:25:45.961694',NULL),(56,64,1,0,3,'2024-05-14 16:36:46.880663',NULL),(57,64,1,0,3,'2024-05-14 16:40:55.247882',NULL),(58,64,1,0,3,'2024-05-14 16:42:23.370682',NULL),(59,64,1,0,3,'2024-05-14 17:28:32.148473',NULL),(60,64,1,0,3,'2024-05-14 17:32:21.904083',NULL),(61,65,1,0,3,'2024-05-14 17:37:35.122996',NULL),(62,65,1,0,2,'2024-05-14 17:38:16.373775','2024-05-14 17:47:42.156955'),(63,65,1,0,2,'2024-05-14 17:48:30.914004','2024-05-14 17:48:50.288634'),(64,65,1,0,2,'2024-05-14 18:07:10.134707','2024-05-14 18:08:25.283611'),(65,65,1,1,3,'2024-05-14 18:09:13.152169',NULL),(66,65,2,0,3,'2024-05-14 18:11:03.248666',NULL),(67,65,2,0,3,'2024-05-14 18:11:17.978765',NULL),(68,65,2,0,3,'2024-05-14 18:11:46.406255',NULL),(69,65,1,0,2,'2024-05-14 18:13:16.559833','2024-05-14 18:13:44.071545'),(70,65,2,0,2,'2024-05-14 18:13:52.894377','2024-05-14 18:14:35.295575'),(71,65,1,1,2,'2024-05-14 18:13:52.897812','2024-05-14 18:14:38.363104'),(72,65,2,1,2,'2024-05-14 18:14:46.407688','2024-05-14 18:14:52.485441'),(73,69,1,0,1,'2024-05-16 12:53:58.788931',NULL),(74,70,1,0,2,'2024-05-16 14:40:57.449782','2024-05-16 14:45:32.774390'),(75,71,1,0,2,'2024-05-16 14:48:00.225464','2024-05-16 14:59:52.377754'),(76,72,1,0,2,'2024-05-16 15:19:35.534172','2024-05-16 15:21:06.081669'),(77,72,1,1,2,'2024-05-16 15:21:19.738176','2024-05-16 15:21:39.380616'),(78,67,1,0,3,'2024-05-16 15:28:32.509012',NULL),(79,72,9,0,2,'2024-05-16 16:21:58.565128','2024-05-16 16:42:24.418702'),(80,72,9,1,0,'2024-05-16 16:42:35.687810',NULL),(81,74,12,0,3,'2024-05-16 17:04:38.993153',NULL),(82,73,6,0,2,'2024-05-16 19:58:03.893019','2024-05-16 20:15:38.864540'),(83,77,11,0,2,'2024-05-16 20:30:07.866584','2024-05-16 20:31:55.155314'),(84,66,4,0,3,'2024-05-17 11:17:05.719584',NULL);
/*!40000 ALTER TABLE `order_game` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `order_game_status_log`
--

DROP TABLE IF EXISTS `order_game_status_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `order_game_status_log` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `order_game_id` bigint DEFAULT NULL,
  `before_status` int NOT NULL,
  `after_status` int NOT NULL,
  `changed_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKps2iblfbq17edi3oxh1u2c55w` (`order_game_id`),
  CONSTRAINT `FKps2iblfbq17edi3oxh1u2c55w` FOREIGN KEY (`order_game_id`) REFERENCES `order_game` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=145 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `order_game_status_log`
--

LOCK TABLES `order_game_status_log` WRITE;
/*!40000 ALTER TABLE `order_game_status_log` DISABLE KEYS */;
INSERT INTO `order_game_status_log` (`id`, `order_game_id`, `before_status`, `after_status`, `changed_at`) VALUES (1,1,0,0,'2024-05-06 07:36:42.349907'),(2,1,0,1,'2024-05-06 07:38:30.297810'),(3,1,1,2,'2024-05-06 07:39:13.144828'),(4,2,0,0,'2024-05-06 11:42:30.036291'),(5,3,0,0,'2024-05-06 11:46:49.321088'),(6,4,0,0,'2024-05-07 16:21:40.089693'),(7,5,0,0,'2024-05-07 16:24:58.254799'),(8,6,0,0,'2024-05-07 16:25:13.126961'),(9,7,0,0,'2024-05-07 16:25:27.633830'),(10,8,0,0,'2024-05-07 16:26:45.704729'),(11,9,0,0,'2024-05-07 16:27:02.235425'),(12,10,0,0,'2024-05-07 16:28:26.950151'),(13,11,0,0,'2024-05-07 16:30:07.795331'),(14,12,0,0,'2024-05-07 16:31:40.797054'),(15,13,0,0,'2024-05-07 16:31:45.578446'),(16,14,0,0,'2024-05-07 17:17:58.911406'),(17,15,0,0,'2024-05-08 12:18:48.146130'),(18,16,0,0,'2024-05-08 12:18:54.152911'),(19,17,0,0,'2024-05-08 12:30:11.434177'),(20,18,0,0,'2024-05-08 12:30:13.573148'),(21,19,0,0,'2024-05-08 12:51:44.960509'),(22,20,0,0,'2024-05-08 12:51:48.285122'),(23,21,0,0,'2024-05-08 13:12:25.195575'),(24,22,0,0,'2024-05-08 13:12:31.535401'),(25,23,0,0,'2024-05-08 13:12:43.825067'),(26,24,0,0,'2024-05-08 13:12:49.309020'),(27,12,0,1,'2024-05-10 14:22:10.553371'),(28,12,0,1,'2024-05-10 15:28:39.317951'),(29,13,0,2,'2024-05-10 15:28:39.502881'),(30,12,1,2,'2024-05-10 15:34:05.783859'),(31,13,1,2,'2024-05-10 15:42:50.999408'),(32,3,0,1,'2024-05-10 17:09:08.565802'),(33,3,1,2,'2024-05-10 17:09:42.072304'),(34,25,0,0,'2024-05-13 15:36:10.338755'),(35,26,0,0,'2024-05-13 15:45:22.301972'),(36,27,0,0,'2024-05-13 15:56:59.056483'),(37,28,0,0,'2024-05-13 15:57:07.249711'),(38,29,0,0,'2024-05-13 16:01:23.838266'),(39,30,0,0,'2024-05-13 16:01:28.679538'),(40,31,0,0,'2024-05-14 13:01:32.362049'),(41,32,0,0,'2024-05-14 13:10:16.878203'),(42,33,0,0,'2024-05-14 13:31:20.514926'),(43,34,0,0,'2024-05-14 13:31:23.225007'),(44,35,0,0,'2024-05-14 13:37:07.645887'),(45,36,0,0,'2024-05-14 13:43:38.193503'),(46,37,0,0,'2024-05-14 13:54:03.168906'),(47,38,0,0,'2024-05-14 13:55:26.547313'),(48,39,0,0,'2024-05-14 13:56:24.922503'),(49,40,0,0,'2024-05-14 13:59:12.523668'),(50,41,0,0,'2024-05-14 14:54:49.229351'),(51,42,0,0,'2024-05-14 15:01:00.943972'),(52,43,0,0,'2024-05-14 15:42:06.182187'),(53,44,0,0,'2024-05-14 15:43:34.452537'),(54,44,0,3,'2024-05-14 15:45:34.660696'),(55,43,0,3,'2024-05-14 15:45:53.094654'),(56,45,0,0,'2024-05-14 15:46:05.928449'),(57,46,0,0,'2024-05-14 15:46:43.194601'),(58,46,0,1,'2024-05-14 15:47:15.871104'),(59,46,1,2,'2024-05-14 15:47:24.088820'),(60,47,0,0,'2024-05-14 15:47:40.416751'),(61,47,0,2,'2024-05-14 15:47:53.203811'),(62,48,0,0,'2024-05-14 15:48:17.165913'),(63,48,0,2,'2024-05-14 15:48:25.937321'),(64,49,0,0,'2024-05-14 15:48:37.550735'),(65,49,0,2,'2024-05-14 15:53:51.159216'),(66,50,0,0,'2024-05-14 16:03:20.280452'),(67,50,0,2,'2024-05-14 16:03:48.863376'),(68,52,0,0,'2024-05-14 16:04:00.934594'),(69,51,0,0,'2024-05-14 16:04:00.935374'),(70,53,0,0,'2024-05-14 16:04:21.510832'),(71,51,0,3,'2024-05-14 16:08:31.703006'),(72,53,0,2,'2024-05-14 16:09:08.715683'),(73,54,0,0,'2024-05-14 16:09:35.145302'),(74,54,0,3,'2024-05-14 16:10:07.234944'),(75,55,0,0,'2024-05-14 16:25:46.012647'),(76,55,0,3,'2024-05-14 16:33:03.759785'),(77,56,0,0,'2024-05-14 16:36:46.883324'),(78,56,0,3,'2024-05-14 16:39:17.026238'),(79,57,0,0,'2024-05-14 16:40:55.250810'),(80,57,0,3,'2024-05-14 16:42:15.445359'),(81,58,0,0,'2024-05-14 16:42:23.372919'),(82,58,0,3,'2024-05-14 16:42:30.478556'),(83,2,0,1,'2024-05-14 17:10:31.647871'),(84,52,0,3,'2024-05-14 17:17:45.843487'),(85,59,0,0,'2024-05-14 17:28:32.150745'),(86,59,0,3,'2024-05-14 17:28:36.271518'),(87,59,3,3,'2024-05-14 17:28:38.188552'),(88,60,0,0,'2024-05-14 17:32:21.906400'),(89,60,0,3,'2024-05-14 17:32:26.593731'),(90,61,0,0,'2024-05-14 17:37:35.125181'),(91,61,0,3,'2024-05-14 17:37:39.181010'),(92,62,0,0,'2024-05-14 17:38:16.376065'),(93,62,0,2,'2024-05-14 17:47:42.151996'),(94,63,0,0,'2024-05-14 17:48:30.922895'),(95,63,0,2,'2024-05-14 17:48:50.285566'),(96,2,1,2,'2024-05-14 18:00:34.038671'),(97,64,0,0,'2024-05-14 18:07:10.136898'),(98,64,0,2,'2024-05-14 18:08:25.281409'),(99,65,0,0,'2024-05-14 18:09:13.154546'),(100,65,0,3,'2024-05-14 18:10:24.261924'),(101,66,0,0,'2024-05-14 18:11:03.251300'),(102,66,0,3,'2024-05-14 18:11:08.253595'),(103,67,0,0,'2024-05-14 18:11:17.980896'),(104,67,0,3,'2024-05-14 18:11:34.451257'),(105,68,0,0,'2024-05-14 18:11:46.408429'),(106,68,0,3,'2024-05-14 18:11:49.388003'),(107,69,0,0,'2024-05-14 18:13:16.600999'),(108,69,0,2,'2024-05-14 18:13:44.069136'),(109,70,0,0,'2024-05-14 18:13:52.896957'),(110,71,0,0,'2024-05-14 18:13:52.900346'),(111,70,0,2,'2024-05-14 18:14:35.293555'),(112,71,0,2,'2024-05-14 18:14:38.360808'),(113,72,0,0,'2024-05-14 18:14:46.410544'),(114,72,0,2,'2024-05-14 18:14:52.483374'),(115,73,0,0,'2024-05-16 12:53:58.795371'),(116,23,0,3,'2024-05-16 13:03:12.448959'),(117,24,0,3,'2024-05-16 13:03:13.615285'),(118,73,0,1,'2024-05-16 13:05:09.963406'),(119,74,0,0,'2024-05-16 14:40:57.454191'),(120,74,0,1,'2024-05-16 14:45:29.415794'),(121,74,1,2,'2024-05-16 14:45:32.772638'),(122,75,0,0,'2024-05-16 14:48:00.227475'),(123,75,0,1,'2024-05-16 14:58:02.183107'),(124,75,1,2,'2024-05-16 14:59:52.375762'),(125,76,0,0,'2024-05-16 15:19:35.536417'),(126,76,0,2,'2024-05-16 15:21:06.078316'),(127,77,0,0,'2024-05-16 15:21:19.740248'),(128,77,0,2,'2024-05-16 15:21:39.378944'),(129,78,0,0,'2024-05-16 15:28:32.521128'),(130,78,0,3,'2024-05-16 15:28:36.667570'),(131,79,0,0,'2024-05-16 16:21:58.568921'),(132,79,0,1,'2024-05-16 16:36:58.610009'),(133,79,1,2,'2024-05-16 16:42:24.415749'),(134,80,0,0,'2024-05-16 16:42:35.694948'),(135,81,0,0,'2024-05-16 17:04:38.998112'),(136,81,0,3,'2024-05-16 19:15:36.329423'),(137,82,0,0,'2024-05-16 19:58:03.899374'),(138,82,0,1,'2024-05-16 20:14:23.283693'),(139,82,1,2,'2024-05-16 20:15:38.862938'),(140,83,0,0,'2024-05-16 20:30:07.868570'),(141,83,0,1,'2024-05-16 20:30:49.621149'),(142,83,1,2,'2024-05-16 20:31:55.153988'),(143,84,0,0,'2024-05-17 11:17:05.763821'),(144,84,0,3,'2024-05-17 11:17:08.902577');
/*!40000 ALTER TABLE `order_game_status_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `order_menu`
--

DROP TABLE IF EXISTS `order_menu`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `order_menu` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `customer_id` int DEFAULT NULL,
  `order_status` int NOT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `delivered_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKi7hcklrx4d9reh9nw8hlxdj79` (`customer_id`),
  CONSTRAINT `FKi7hcklrx4d9reh9nw8hlxdj79` FOREIGN KEY (`customer_id`) REFERENCES `customer` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=34 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `order_menu`
--

LOCK TABLES `order_menu` WRITE;
/*!40000 ALTER TABLE `order_menu` DISABLE KEYS */;
INSERT INTO `order_menu` (`id`, `customer_id`, `order_status`, `created_at`, `delivered_at`) VALUES (2,4,3,'2024-05-03 10:27:51.769582','2024-05-16 20:31:55.148468'),(3,4,3,'2024-05-03 10:29:58.774297','2024-05-03 12:57:57.999608'),(4,3,1,'2024-05-06 11:53:22.562791',NULL),(5,16,3,'2024-05-07 16:58:52.973552','2024-05-10 15:42:50.401029'),(6,18,0,'2024-05-07 16:59:27.063095',NULL),(7,24,0,'2024-05-07 17:18:15.000560',NULL),(8,25,0,'2024-05-07 17:42:41.193606',NULL),(9,26,0,'2024-05-07 17:48:50.551224',NULL),(10,27,0,'2024-05-07 17:50:59.330370',NULL),(11,28,0,'2024-05-07 17:55:50.227297',NULL),(12,29,0,'2024-05-07 17:57:47.022781',NULL),(13,29,0,'2024-05-07 17:59:12.430139',NULL),(14,29,0,'2024-05-07 17:59:21.779911',NULL),(15,29,0,'2024-05-07 17:59:29.501593',NULL),(16,29,0,'2024-05-07 17:59:35.071009',NULL),(17,30,0,'2024-05-08 10:49:50.480014',NULL),(18,30,0,'2024-05-08 10:50:01.659406',NULL),(19,30,0,'2024-05-08 10:50:12.164838',NULL),(20,37,0,'2024-05-08 12:53:53.866581',NULL),(21,46,0,'2024-05-09 15:43:55.728760',NULL),(22,47,0,'2024-05-13 09:36:53.811554',NULL),(23,51,0,'2024-05-13 13:13:11.792582',NULL),(24,50,0,'2024-05-13 13:22:03.162802',NULL),(25,54,0,'2024-05-13 15:11:55.575906',NULL),(26,54,0,'2024-05-13 15:14:03.125240',NULL),(27,54,0,'2024-05-13 15:35:53.146284',NULL),(28,64,0,'2024-05-14 16:47:14.136914',NULL),(29,65,0,'2024-05-14 17:45:43.543977',NULL),(30,65,0,'2024-05-14 17:46:04.777891',NULL),(31,67,0,'2024-05-16 15:40:19.306427',NULL),(32,74,0,'2024-05-16 19:15:52.067012',NULL),(33,66,0,'2024-05-17 12:13:25.857270',NULL);
/*!40000 ALTER TABLE `order_menu` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `order_menu_detail`
--

DROP TABLE IF EXISTS `order_menu_detail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `order_menu_detail` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `menu_id` int DEFAULT NULL,
  `order_menu_id` bigint DEFAULT NULL,
  `quantity` int NOT NULL,
  `unit_price` int NOT NULL,
  `total_price` int NOT NULL,
  PRIMARY KEY (`id`),
  KEY `FKfrjqv1saig4cqalrf50qj0pi7` (`menu_id`),
  KEY `FKh58x09597sk2myb8dd71bh15o` (`order_menu_id`),
  CONSTRAINT `FKfrjqv1saig4cqalrf50qj0pi7` FOREIGN KEY (`menu_id`) REFERENCES `menu` (`id`),
  CONSTRAINT `FKh58x09597sk2myb8dd71bh15o` FOREIGN KEY (`order_menu_id`) REFERENCES `order_menu` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=64 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `order_menu_detail`
--

LOCK TABLES `order_menu_detail` WRITE;
/*!40000 ALTER TABLE `order_menu_detail` DISABLE KEYS */;
INSERT INTO `order_menu_detail` (`id`, `menu_id`, `order_menu_id`, `quantity`, `unit_price`, `total_price`) VALUES (1,1,3,1,1000,1000),(2,2,3,2,1000,2000),(3,3,3,3,1000,3000),(4,1,4,1,1000,1000),(5,1,5,1,1000,1000),(6,2,5,1,1000,1000),(7,1,6,2,1000,2000),(8,1,7,2,1000,2000),(9,1,8,2,1000,2000),(10,2,8,1,1000,1000),(11,3,8,1,1000,1000),(12,1,9,1,1000,1000),(13,2,9,1,1000,1000),(14,3,9,2,1000,2000),(15,1,10,2,1000,2000),(16,2,10,1,1000,1000),(17,1,11,1,1000,1000),(18,2,11,1,1000,1000),(19,3,11,1,1000,1000),(20,1,12,2,1000,2000),(21,3,12,1,1000,1000),(22,2,12,1,1000,1000),(23,1,13,2,1000,2000),(24,2,13,1,1000,1000),(25,1,14,1,1000,1000),(26,1,15,1,1000,1000),(27,1,16,2,1000,2000),(28,2,16,1,1000,1000),(29,1,17,2,1000,2000),(30,2,17,1,1000,1000),(31,3,17,1,1000,1000),(32,1,18,1,1000,1000),(33,2,19,3,1000,3000),(34,3,19,1,1000,1000),(35,1,20,1,1000,1000),(36,2,21,1,1000,1000),(37,1,21,1,1000,1000),(38,1,23,1,1000,1000),(39,3,23,1,1000,1000),(40,2,23,1,1000,1000),(41,2,24,1,1000,1000),(42,1,25,1,1000,1000),(43,1,26,1,1000,1000),(44,2,26,1,1000,1000),(45,3,26,1,1000,1000),(46,1,27,2,1000,2000),(47,2,27,1,1000,1000),(48,1,28,1,1000,1000),(49,2,28,1,1000,1000),(50,1,29,1,1000,1000),(51,2,29,1,1000,1000),(52,3,29,1,1000,1000),(53,1,30,1,1000,1000),(54,2,30,1,1000,1000),(55,1,31,1,1000,1000),(56,1,32,3,1000,3000),(57,7,33,1,5500,5500),(58,6,33,1,5500,5500),(59,1,33,1,4500,4500),(60,2,33,2,2000,4000),(61,8,33,1,5000,5000),(62,9,33,1,6000,6000),(63,3,33,1,3000,3000);
/*!40000 ALTER TABLE `order_menu_detail` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `order_menu_status_log`
--

DROP TABLE IF EXISTS `order_menu_status_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `order_menu_status_log` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `order_menu_id` bigint DEFAULT NULL,
  `before_status` int NOT NULL,
  `after_status` int NOT NULL,
  `changed_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKatmw07yjm8p1duu4wuxeermc1` (`order_menu_id`),
  CONSTRAINT `FKatmw07yjm8p1duu4wuxeermc1` FOREIGN KEY (`order_menu_id`) REFERENCES `order_menu` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=48 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `order_menu_status_log`
--

LOCK TABLES `order_menu_status_log` WRITE;
/*!40000 ALTER TABLE `order_menu_status_log` DISABLE KEYS */;
INSERT INTO `order_menu_status_log` (`id`, `order_menu_id`, `before_status`, `after_status`, `changed_at`) VALUES (1,3,0,1,'2024-05-03 10:29:58.851288'),(2,3,0,1,'2024-05-03 12:55:59.597640'),(3,3,1,3,'2024-05-03 12:57:57.999608'),(4,4,0,0,'2024-05-06 11:53:22.698875'),(5,5,0,0,'2024-05-07 16:58:52.983422'),(6,6,0,0,'2024-05-07 16:59:27.066545'),(7,7,0,0,'2024-05-07 17:18:15.003788'),(8,8,0,0,'2024-05-07 17:42:41.209285'),(9,9,0,0,'2024-05-07 17:48:50.558992'),(10,10,0,0,'2024-05-07 17:50:59.336302'),(11,11,0,0,'2024-05-07 17:55:50.234146'),(12,12,0,0,'2024-05-07 17:57:47.030176'),(13,13,0,0,'2024-05-07 17:59:12.435289'),(14,14,0,0,'2024-05-07 17:59:21.783507'),(15,15,0,0,'2024-05-07 17:59:29.505470'),(16,16,0,0,'2024-05-07 17:59:35.075784'),(17,17,0,0,'2024-05-08 10:49:50.486979'),(18,18,0,0,'2024-05-08 10:50:01.663287'),(19,19,0,0,'2024-05-08 10:50:12.169914'),(20,20,0,0,'2024-05-08 12:53:53.870589'),(21,2,3,2,'2024-05-09 09:41:46.608647'),(22,21,0,0,'2024-05-09 15:43:55.741123'),(23,2,3,2,'2024-05-09 16:17:55.156985'),(24,2,2,2,'2024-05-09 16:20:44.422526'),(25,2,2,2,'2024-05-09 16:22:49.504252'),(26,2,2,2,'2024-05-09 16:23:16.441300'),(27,2,2,2,'2024-05-09 16:25:48.756823'),(28,2,2,2,'2024-05-09 16:52:23.288424'),(29,4,0,1,'2024-05-10 14:22:10.114902'),(30,5,0,1,'2024-05-10 15:28:39.209241'),(31,5,1,2,'2024-05-10 15:34:05.697791'),(33,5,2,3,'2024-05-10 15:42:50.418471'),(34,22,0,0,'2024-05-13 09:36:53.818592'),(35,23,0,0,'2024-05-13 13:13:11.848262'),(36,24,0,0,'2024-05-13 13:22:03.167667'),(37,25,0,0,'2024-05-13 15:11:55.586615'),(38,26,0,0,'2024-05-13 15:14:03.135921'),(39,27,0,0,'2024-05-13 15:35:53.151738'),(40,28,0,0,'2024-05-14 16:47:14.147515'),(41,29,0,0,'2024-05-14 17:45:43.601830'),(42,30,0,0,'2024-05-14 17:46:04.784565'),(43,31,0,0,'2024-05-16 15:40:19.341034'),(44,32,0,0,'2024-05-16 19:15:52.081605'),(45,2,2,3,'2024-05-16 20:15:38.858679'),(46,2,3,3,'2024-05-16 20:31:55.148941'),(47,33,0,0,'2024-05-17 12:13:25.877050');
/*!40000 ALTER TABLE `order_menu_status_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `room`
--

DROP TABLE IF EXISTS `room`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `room` (
  `id` int NOT NULL AUTO_INCREMENT,
  `store_id` int NOT NULL,
  `coordinatex` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `coordinatey` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci NOT NULL,
  `room_number` int NOT NULL,
  `fcm_token` varchar(255) COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `iot_id` varchar(255) COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKpviy6dvvot2ne7jl43j603mn3` (`store_id`),
  CONSTRAINT `FKpviy6dvvot2ne7jl43j603mn3` FOREIGN KEY (`store_id`) REFERENCES `store` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `room`
--

LOCK TABLES `room` WRITE;
/*!40000 ALTER TABLE `room` DISABLE KEYS */;
INSERT INTO `room` (`id`, `store_id`, `coordinatex`, `coordinatey`, `room_number`, `fcm_token`, `iot_id`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,1,'0.43','9.32',1,'string','string','2024-04-25 14:15:02.532177','2024-05-16 19:54:09.039345',NULL),(2,1,'111','222',2,NULL,NULL,'2024-04-25 14:45:07.799348','2024-04-25 15:07:43.866270',NULL),(3,1,'2.00','2.57',0,'string','string','2024-05-09 16:19:40.108498','2024-05-16 19:54:13.233450',NULL),(4,1,'0.0','0.0',255,NULL,NULL,'2024-05-10 13:15:05.750299','2024-05-10 16:52:30.548163',NULL);
/*!40000 ALTER TABLE `room` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `stock`
--

DROP TABLE IF EXISTS `stock`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `stock` (
  `id` int NOT NULL AUTO_INCREMENT,
  `store_id` int DEFAULT NULL,
  `game_id` int DEFAULT NULL,
  `is_available` int NOT NULL,
  `stock_location` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKmjukt6ibaqa0vo6dotmw4md2w` (`game_id`),
  KEY `FKa9un5jymd7dqr3orbkt9fa74b` (`store_id`),
  CONSTRAINT `FKa9un5jymd7dqr3orbkt9fa74b` FOREIGN KEY (`store_id`) REFERENCES `store` (`id`),
  CONSTRAINT `FKmjukt6ibaqa0vo6dotmw4md2w` FOREIGN KEY (`game_id`) REFERENCES `game` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=24 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `stock`
--

LOCK TABLES `stock` WRITE;
/*!40000 ALTER TABLE `stock` DISABLE KEYS */;
INSERT INTO `stock` (`id`, `store_id`, `game_id`, `is_available`, `stock_location`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,1,1,1,'A-10',NULL,'2024-05-16 16:07:42.149862','2024-05-16 16:07:42.149366'),(2,1,9,1,'A-10','2024-05-04 17:47:20.984151','2024-05-16 16:07:39.580113','2024-05-16 16:07:39.579616'),(3,1,1,1,'A-01','2024-05-16 16:02:50.482097','2024-05-16 16:07:37.723792','2024-05-16 16:07:37.723304'),(4,1,11,1,'A-02','2024-05-16 16:03:28.749374','2024-05-17 11:17:08.907439',NULL),(5,1,12,1,'A-02','2024-05-16 16:03:34.558362','2024-05-16 16:03:34.558362',NULL),(6,1,13,0,'A-02','2024-05-16 16:03:38.904158','2024-05-16 19:58:03.907063',NULL),(7,1,14,1,'A-02','2024-05-16 16:04:02.811744','2024-05-16 16:04:02.811744',NULL),(8,1,15,1,'A-03','2024-05-16 16:04:08.910610','2024-05-16 16:04:08.910610',NULL),(9,1,16,0,'A-04','2024-05-16 16:04:13.438270','2024-05-16 16:21:58.576349',NULL),(10,1,17,1,'A-07','2024-05-16 16:04:22.884019','2024-05-16 16:04:22.884019',NULL),(11,1,18,0,'A-08','2024-05-16 16:04:27.519465','2024-05-16 20:30:07.876118',NULL),(12,1,19,1,'A-09','2024-05-16 16:04:32.820831','2024-05-16 19:15:36.366806',NULL),(13,1,20,1,'A-10','2024-05-16 16:04:38.269263','2024-05-16 16:04:38.269263',NULL),(14,1,21,1,'B-01','2024-05-16 16:04:46.569110','2024-05-16 16:04:46.569110',NULL),(15,1,22,1,'B-02','2024-05-16 16:04:51.978932','2024-05-16 16:04:51.978932',NULL),(16,1,23,1,'B-03','2024-05-16 16:04:56.447808','2024-05-16 16:04:56.447808',NULL),(17,1,24,1,'B-04','2024-05-16 16:05:00.663410','2024-05-16 16:05:00.663410',NULL),(18,1,25,1,'B-05','2024-05-16 16:05:05.109795','2024-05-16 16:05:05.109795',NULL),(19,1,26,1,'B-06','2024-05-16 16:05:10.047291','2024-05-16 16:05:10.047291',NULL),(20,1,27,1,'B-07','2024-05-16 16:05:15.206646','2024-05-16 16:05:15.206646',NULL),(21,1,28,1,'B-08','2024-05-16 16:05:19.103109','2024-05-16 16:05:19.103109',NULL),(22,1,29,1,'B-09','2024-05-16 16:05:23.055132','2024-05-16 16:05:23.055132',NULL),(23,1,30,1,'B-10','2024-05-16 16:05:28.737062','2024-05-16 16:05:28.737062',NULL);
/*!40000 ALTER TABLE `stock` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `store`
--

DROP TABLE IF EXISTS `store`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `store` (
  `id` int NOT NULL AUTO_INCREMENT,
  `store_name` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `hour_fee` int NOT NULL,
  `latitude` varchar(20) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `longitude` varchar(20) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `address` varchar(30) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `hours` varchar(20) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `phone` varchar(13) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=10 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `store`
--

LOCK TABLES `store` WRITE;
/*!40000 ALTER TABLE `store` DISABLE KEYS */;
INSERT INTO `store` (`id`, `store_name`, `hour_fee`, `latitude`, `longitude`, `address`, `hours`, `phone`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,'멀티캠퍼스 역삼점',3000,'37.5012647456244','127.03958123605','역삼역 1번출구 역삼 멀티캠퍼스 15층','11:00 ~ 22:00','000-0000-0000','2024-04-25 11:08:12.828880','2024-05-10 17:26:15.811099',NULL),(2,'깊은 저 바닷속 파인애플점',3000,'37.4992647456244','129.13958123605','뚱이네, 징징이네 옆집','11:00 ~ 22:00','000-0000-0000','2024-04-25 11:13:23.398668','2024-04-25 11:13:43.920038',NULL),(3,'강남역점',3000,'37.4980647456244','127.02875123605','강남역 1번출구','11:00 ~ 22:00','010-0000-0000','2024-05-10 16:32:39.757681','2024-05-10 16:34:34.595312',NULL),(4,'멀티캠퍼스 역삼점',1,'37.5012647456244','127.03958123605','역삼역 1번출구 역삼 멀티캠퍼스 15층','11:00 ~ 22:00','000-0000-0000','2024-05-10 16:34:42.621276','2024-05-10 16:34:42.621276','2024-05-17 13:32:25.000000'),(5,'깊은 저 바닷속 파인애플점',2,'37.4992647456244','129.13958123605','뚱이네, 징징이네 옆집','11:00 ~ 22:00','000-0000-0000','2024-05-10 16:35:52.097895','2024-05-10 16:35:52.097895','2024-05-10 16:35:52.097895'),(6,'강남역점',3,'37.4980647456244','127.02875123605','강남역 1번출구','11:00 ~ 22:00','010-0000-0000','2024-05-10 16:36:42.125701','2024-05-10 16:36:42.125701','2024-05-10 16:36:42.125701');
/*!40000 ALTER TABLE `store` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `theme`
--

DROP TABLE IF EXISTS `theme`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `theme` (
  `id` int NOT NULL AUTO_INCREMENT,
  `theme_type` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `theme_video_url` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `theme`
--

LOCK TABLES `theme` WRITE;
/*!40000 ALTER TABLE `theme` DISABLE KEYS */;
INSERT INTO `theme` (`id`, `theme_type`, `theme_video_url`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,'sample',NULL,NULL,NULL,NULL),(2,'testType','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/testType/3c4a48a5-f899-4acb-8ada-90afbd0b1074_stay.webm','2024-05-03 11:06:32.131783','2024-05-03 13:28:11.512588','2024-05-03 13:28:11.495590'),(3,'testType','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/testType/c8dc7fe9-f77d-4db9-9b7f-b1d3675604a0_stay.webm','2024-05-03 13:11:12.963975','2024-05-03 13:11:12.963975',NULL),(4,'west','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/west/ac28a7a3-ffbb-44a5-8af9-e915a5648853_ssafy.mp4','2024-05-16 11:12:16.187991','2024-05-16 11:12:16.187991',NULL),(5,'middle','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/middle/4b269829-5b57-48ad-bf9b-972a07833d2c_ssafy.mp4','2024-05-16 11:12:28.183258','2024-05-16 11:12:28.183258',NULL),(6,'puzzle','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/puzzle/492ad3d3-9a7f-4eb5-9ab9-68d1df0bba16_ssafy.mp4','2024-05-16 11:12:37.734970','2024-05-16 11:12:37.734970',NULL),(7,'jungle','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/jungle/42b2ac02-eb79-466d-ba67-c0f9ff9770a0_ssafy.mp4','2024-05-16 11:12:44.587731','2024-05-16 11:12:44.587731',NULL),(8,'space','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/space/1380479b-26bc-4c76-b096-1ab3bd6958e1_ssafy.mp4','2024-05-16 11:12:50.569262','2024-05-16 11:12:50.569262',NULL),(9,'horror','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/horror/7413c663-d671-46ac-a3a8-5ef3d18241fb_ssafy.mp4','2024-05-16 11:12:57.775038','2024-05-16 11:12:57.775038',NULL),(10,'ocean','https://accio-isegye.s3.ap-northeast-2.amazonaws.com/gameTheme/ocean/8ae04ade-e89f-4055-acee-bf713ee5b112_ssafy.mp4','2024-05-16 11:13:03.703708','2024-05-16 11:13:03.703708',NULL);
/*!40000 ALTER TABLE `theme` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `turtle`
--

DROP TABLE IF EXISTS `turtle`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `turtle` (
  `id` int NOT NULL AUTO_INCREMENT,
  `store_id` int DEFAULT NULL,
  `is_working` int NOT NULL,
  `created_at` datetime(6) DEFAULT NULL,
  `updated_at` datetime(6) DEFAULT NULL,
  `deleted_at` datetime(6) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FK26x2bj6hp63i6ubq4a8blylku` (`store_id`),
  CONSTRAINT `FK26x2bj6hp63i6ubq4a8blylku` FOREIGN KEY (`store_id`) REFERENCES `store` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `turtle`
--

LOCK TABLES `turtle` WRITE;
/*!40000 ALTER TABLE `turtle` DISABLE KEYS */;
INSERT INTO `turtle` (`id`, `store_id`, `is_working`, `created_at`, `updated_at`, `deleted_at`) VALUES (1,1,0,'2024-05-09 09:31:55.950397','2024-05-16 20:38:28.961654',NULL),(2,1,0,'2024-05-09 15:10:28.663458','2024-05-16 19:47:26.148084',NULL),(3,1,0,'2024-05-14 17:36:22.342917','2024-05-14 17:36:22.342917',NULL),(4,1,0,'2024-05-14 17:49:09.111433','2024-05-14 17:49:09.111433',NULL);
/*!40000 ALTER TABLE `turtle` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `turtle_log`
--

DROP TABLE IF EXISTS `turtle_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `turtle_log` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `turtle_id` int DEFAULT NULL,
  `command_type` int NOT NULL,
  `command_start_time` datetime(6) DEFAULT NULL,
  `command_end_time` datetime(6) DEFAULT NULL,
  `is_success` int NOT NULL,
  `log_message` varchar(255) CHARACTER SET utf8mb3 COLLATE utf8mb3_unicode_ci DEFAULT NULL,
  `order_menu_id` bigint DEFAULT NULL,
  `order_game_id` bigint DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKoi2f50g2qn6m56i2ldl6u3ebh` (`order_game_id`),
  KEY `FK5l95b97xpshjjtb2caoco4i4i` (`order_menu_id`),
  KEY `FKcuba1vw0rit7bfddfbeggdnyd` (`turtle_id`),
  CONSTRAINT `FK5l95b97xpshjjtb2caoco4i4i` FOREIGN KEY (`order_menu_id`) REFERENCES `order_menu` (`id`),
  CONSTRAINT `FKcuba1vw0rit7bfddfbeggdnyd` FOREIGN KEY (`turtle_id`) REFERENCES `turtle` (`id`),
  CONSTRAINT `FKoi2f50g2qn6m56i2ldl6u3ebh` FOREIGN KEY (`order_game_id`) REFERENCES `order_game` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=91 DEFAULT CHARSET=utf8mb3 COLLATE=utf8mb3_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `turtle_log`
--

LOCK TABLES `turtle_log` WRITE;
/*!40000 ALTER TABLE `turtle_log` DISABLE KEYS */;
INSERT INTO `turtle_log` (`id`, `turtle_id`, `command_type`, `command_start_time`, `command_end_time`, `is_success`, `log_message`, `order_menu_id`, `order_game_id`) VALUES (1,1,0,'2024-05-09 09:41:46.441334','2024-05-09 14:49:30.849859',1,'{\"turtleLogId\":1}',2,NULL),(2,1,1,'2024-05-09 12:38:38.167024','2024-05-09 14:23:18.345768',1,'{\"turtleLogId\":2}',2,NULL),(3,1,1,'2024-05-09 14:07:08.100590',NULL,0,NULL,2,NULL),(4,1,1,'2024-05-09 14:07:08.927001',NULL,0,NULL,2,NULL),(5,1,1,'2024-05-09 14:08:44.875000',NULL,0,NULL,2,NULL),(6,1,1,'2024-05-09 14:23:17.669687',NULL,0,NULL,2,NULL),(7,1,1,'2024-05-09 14:23:18.424628',NULL,0,NULL,2,NULL),(8,1,1,'2024-05-09 14:49:30.973178','2024-05-09 14:50:29.046348',1,'{\"turtleLogId\":8}',2,NULL),(10,1,0,'2024-05-09 16:17:54.951079',NULL,0,NULL,2,NULL),(11,1,0,'2024-05-09 16:20:44.152959',NULL,0,NULL,2,NULL),(12,1,0,'2024-05-09 16:22:49.317564',NULL,0,NULL,2,NULL),(13,1,0,'2024-05-09 16:23:16.262282',NULL,0,NULL,2,NULL),(14,1,0,'2024-05-09 16:25:48.617927','2024-05-09 16:52:23.221188',1,'StartTurtleOrderRequest(turtleId=1, turtleLogId=14)',2,NULL),(15,1,1,'2024-05-09 16:38:25.242193',NULL,0,NULL,2,NULL),(16,1,1,'2024-05-09 16:52:23.418061',NULL,0,NULL,2,NULL),(17,1,0,'2024-05-10 14:08:32.607184',NULL,0,NULL,4,12),(18,1,0,'2024-05-10 14:08:32.895576',NULL,0,NULL,NULL,13),(19,1,1,'2024-05-10 14:22:11.345753',NULL,0,NULL,4,12),(20,1,0,'2024-05-10 14:47:06.900511','2024-05-10 15:28:39.349864',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 20, \"turtleReceiveLogId\" : 21}',5,12),(21,1,0,'2024-05-10 14:47:07.116466','2024-05-10 15:28:39.584646',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 20, \"turtleReceiveLogId\" : 21}',NULL,13),(22,2,0,'2024-05-10 15:07:42.035482',NULL,0,NULL,2,2),(23,1,1,'2024-05-10 15:28:39.379785','2024-05-10 15:34:06.112692',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 23, \"turtleReceiveLogId\" : 24}',5,12),(24,1,1,'2024-05-10 15:28:39.585642','2024-05-10 15:34:06.292560',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 23, \"turtleReceiveLogId\" : 24}',NULL,13),(25,1,255,'2024-05-10 15:34:06.155801','2024-05-10 15:42:50.813402',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 25, \"turtleReceiveLogId\" : 26}',5,12),(26,1,0,'2024-05-10 15:34:06.333913','2024-05-10 15:42:51.040315',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 25, \"turtleReceiveLogId\" : 26}',NULL,13),(27,1,255,'2024-05-10 15:42:50.878265','2024-05-10 15:57:21.970909',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 27, \"turtleReceiveLogId\" : 28}',5,12),(28,1,255,'2024-05-10 15:42:51.040315','2024-05-10 15:57:21.976912',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 27, \"turtleReceiveLogId\" : 28}',NULL,13),(29,1,0,'2024-05-10 16:53:23.108952',NULL,0,NULL,NULL,2),(30,1,0,'2024-05-10 17:08:59.097652','2024-05-10 17:09:08.570389',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 30, \"turtleReceiveLogId\" : -1}',NULL,3),(31,1,1,'2024-05-10 17:09:08.573268','2024-05-10 17:09:42.088621',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 31, \"turtleReceiveLogId\" : -1}',NULL,3),(32,1,255,'2024-05-10 17:09:42.091038','2024-05-10 17:09:44.069887',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 32, \"turtleReceiveLogId\" : -1}',NULL,3),(33,1,255,'2024-05-10 17:09:44.072243',NULL,0,NULL,NULL,3),(34,1,0,'2024-05-10 17:15:15.608179',NULL,0,NULL,NULL,2),(35,1,0,'2024-05-10 17:31:50.967039',NULL,0,NULL,NULL,2),(36,1,0,'2024-05-14 14:04:20.708886',NULL,0,NULL,NULL,35),(37,1,0,'2024-05-14 14:04:35.236168',NULL,0,NULL,NULL,35),(38,1,0,'2024-05-14 14:04:39.436382',NULL,0,NULL,4,36),(39,1,0,'2024-05-14 14:04:53.539514',NULL,0,NULL,4,35),(40,1,0,'2024-05-14 14:06:39.422791',NULL,0,NULL,NULL,35),(41,1,0,'2024-05-14 14:08:19.532650',NULL,0,NULL,NULL,35),(42,1,0,'2024-05-14 14:10:03.001356',NULL,0,NULL,NULL,2),(43,1,0,'2024-05-14 14:14:14.730355',NULL,0,NULL,4,2),(44,1,0,'2024-05-14 14:21:10.746919',NULL,0,NULL,4,2),(47,1,0,'2024-05-14 15:20:45.084549',NULL,0,NULL,NULL,2),(48,1,0,'2024-05-14 15:24:28.625103',NULL,0,NULL,NULL,2),(49,1,0,'2024-05-14 15:50:24.342562',NULL,0,NULL,NULL,2),(50,1,0,'2024-05-14 15:56:35.157607',NULL,0,NULL,NULL,2),(51,1,0,'2024-05-14 16:10:49.381185',NULL,0,NULL,NULL,2),(53,1,0,'2024-05-14 17:10:04.350911','2024-05-14 17:10:31.649889',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 53, \"turtleReceiveLogId\" : -1}',NULL,2),(54,1,1,'2024-05-14 17:10:31.651949',NULL,0,NULL,NULL,2),(55,1,0,'2024-05-14 17:36:00.118556',NULL,0,NULL,NULL,2),(56,1,0,'2024-05-14 17:46:05.107597','2024-05-14 17:46:38.608893',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 56, \"turtleReceiveLogId\" : -1}',NULL,2),(57,1,1,'2024-05-14 17:46:38.611482',NULL,0,NULL,NULL,2),(58,1,0,'2024-05-14 17:58:12.067367','2024-05-14 17:58:41.960768',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 58, \"turtleReceiveLogId\" : -1}',NULL,2),(59,1,1,'2024-05-14 17:58:41.962592','2024-05-14 18:00:34.054920',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 59, \"turtleReceiveLogId\" : -1}',NULL,2),(60,1,255,'2024-05-14 18:00:34.056894','2024-05-14 18:00:41.369154',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 60, \"turtleReceiveLogId\" : -1}',NULL,2),(61,1,255,'2024-05-14 18:00:41.371326','2024-05-14 18:00:45.008787',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 61, \"turtleReceiveLogId\" : -1}',NULL,2),(62,1,255,'2024-05-14 18:00:45.011463','2024-05-14 18:00:51.864074',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 62, \"turtleReceiveLogId\" : -1}',NULL,2),(63,1,255,'2024-05-14 18:00:51.866455',NULL,0,NULL,NULL,2),(64,1,0,'2024-05-16 13:00:11.747729',NULL,0,NULL,NULL,73),(65,1,0,'2024-05-16 13:04:41.723043','2024-05-16 13:05:09.965639',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 65, \"turtleReceiveLogId\" : -1}',NULL,73),(66,1,1,'2024-05-16 13:05:09.967463',NULL,0,NULL,NULL,73),(67,1,0,'2024-05-16 13:12:50.530806','2024-05-16 13:13:33.817455',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 67, \"turtleReceiveLogId\" : -1}',NULL,73),(68,1,1,'2024-05-16 13:13:33.819388',NULL,0,NULL,NULL,73),(69,1,0,'2024-05-16 14:44:56.210314','2024-05-16 14:45:29.417907',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 69, \"turtleReceiveLogId\" : -1}',NULL,74),(70,1,1,'2024-05-16 14:45:29.419518','2024-05-16 14:45:32.786331',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 70, \"turtleReceiveLogId\" : -1}',NULL,74),(71,1,255,'2024-05-16 14:45:32.789225','2024-05-16 14:45:46.330204',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 71, \"turtleReceiveLogId\" : -1}',NULL,74),(72,1,255,'2024-05-16 14:45:46.332171',NULL,0,NULL,NULL,74),(73,1,0,'2024-05-16 14:57:32.360086','2024-05-16 14:58:02.185825',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 73, \"turtleReceiveLogId\" : -1}',NULL,75),(74,1,1,'2024-05-16 14:58:02.187290','2024-05-16 14:59:52.388453',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 74, \"turtleReceiveLogId\" : -1}',NULL,75),(75,1,255,'2024-05-16 14:59:52.390229',NULL,0,NULL,NULL,75),(76,1,0,'2024-05-16 15:19:49.017820',NULL,0,NULL,NULL,76),(77,1,0,'2024-05-16 16:29:07.230693',NULL,0,NULL,NULL,79),(78,1,0,'2024-05-16 16:35:56.417937','2024-05-16 16:36:58.617985',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 78, \"turtleReceiveLogId\" : -1}',NULL,79),(79,1,1,'2024-05-16 16:36:58.620649',NULL,0,NULL,NULL,79),(80,1,0,'2024-05-16 17:04:53.365737',NULL,0,NULL,NULL,81),(81,1,0,'2024-05-16 17:08:04.558999',NULL,0,NULL,NULL,81),(82,2,0,'2024-05-16 19:21:33.856843',NULL,0,NULL,NULL,1),(83,1,0,'2024-05-16 20:00:35.293623',NULL,0,NULL,2,82),(84,1,0,'2024-05-16 20:08:22.251743',NULL,0,NULL,2,82),(85,1,0,'2024-05-16 20:13:52.809648','2024-05-16 20:14:23.285579',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 85, \"turtleReceiveLogId\" : -1}',2,82),(86,1,1,'2024-05-16 20:14:23.287305','2024-05-16 20:15:38.878955',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 86, \"turtleReceiveLogId\" : -1}',2,82),(87,1,255,'2024-05-16 20:15:38.880872',NULL,0,NULL,2,82),(88,1,0,'2024-05-16 20:30:20.176205','2024-05-16 20:30:49.623884',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 88, \"turtleReceiveLogId\" : -1}',2,83),(89,1,1,'2024-05-16 20:30:49.625560','2024-05-16 20:31:55.165012',1,'{\"turtleId\" : 1, \"turtleOrderLogId\" : 89, \"turtleReceiveLogId\" : -1}',2,83),(90,1,255,'2024-05-16 20:31:55.166906',NULL,0,NULL,2,83);
/*!40000 ALTER TABLE `turtle_log` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2024-05-17 13:34:24
