import { Module } from "@nestjs/common";
import { SupabaseService } from "../supabase.service";
import { HomeAgentsService } from "../home-agents/home-agents.service";
import { DroneVideoAgentController, DroneVideoStaffController } from "./drone-video.controller";
import { DroneVideoStore } from "./drone-video.store";
import { DroneControlController } from "./drone-control.controller";
import { DroneControlGateway } from "./drone-control.gateway";
import { DroneControlGuard } from "./drone-control.guard";

@Module({
  controllers: [DroneVideoAgentController, DroneVideoStaffController, DroneControlController],
  providers: [DroneVideoStore, HomeAgentsService, SupabaseService, DroneControlGateway, DroneControlGuard]
})
export class DroneVideoModule {}
